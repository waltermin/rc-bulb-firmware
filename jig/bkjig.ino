// BKJIG — ESP32-S3 harness for Tuya T1-2S-NL (BK7238) bench work
//
// Wiring:
//   GPIO4  = ESP TX  -> BK RX1
//   GPIO5  = ESP RX  <- BK TX1
//   GPIO6  = CEN     (open-drain: hi-Z idle, low = reset)
//   GPIO7  = P8 btn  (open-drain: hi-Z idle, low = pressed)
//   GPIO15 = P6 LED  (input, BK drives)
//   GPIO16 = P9 sw   (input, BK drives)
//
// Modes:
//   Command mode (default): newline-terminated text commands, see HELP.
//   Bridge mode: transparent USB<->UART1 forwarding until next reset.
//     FLASHMODE arms an NVS flag; the port-open auto-reset boots into
//     bridge mode with a CEN pulse at T+600ms to catch the BK BootROM.

#include <Preferences.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <esp_wifi.h>

#define PIN_TX_TO_BK 4
#define PIN_RX_FROM_BK 5
#define PIN_CEN 6
#define PIN_BTN 7
#define PIN_LED 15
#define PIN_SW 16

#define VERSION "BKJIG v1.1"

Preferences prefs;
uint32_t bkBaud = 115200;

// ---- RC-protocol beacon TX (jig-as-base-station) ----------------------------
// Vendor IE: OUI 52:43:68, single IE, per rc-bulb-firmware spec.txt.

static uint8_t txChannel = 11;
static bool wifiUp = false;
static wifi_interface_t g_txIf = WIFI_IF_STA;  // beacon TX interface (AP when softAP up)
static bool g_passthrough = false;  // echo BK UART1 -> USB (see switch logs during tests)

// ---- DFU2 test server: softAP + TCP server serving an RBL to the switch ------
static uint8_t *g_rbl = NULL;      // RBL image (in PSRAM), served over TCP
static uint32_t g_rblSize = 0;
static uint8_t g_rblSha[32];       // sha256(RBL) for the DFU2 header
static uint8_t g_dfuBuildId[8];    // advertised/served build id
static WiFiServer g_dfuServer(3333);
static bool g_dfuServerActive = false;
static uint8_t relayBits[32];    // 0x40 RelaysUpdate bitfield state
static uint8_t ledBits[32];      // 0x41 RelaysLedUpdate bitfield state
static uint32_t autoPeriodMs = 0;  // 0 = auto-broadcast off
static uint32_t autoLastMs = 0;

static void wifiTxInit() {
  if (wifiUp) return;
  WiFi.mode(WIFI_STA);
  g_txIf = WIFI_IF_STA;
  esp_wifi_set_promiscuous(false);
  esp_wifi_set_channel(txChannel, WIFI_SECOND_CHAN_NONE);
  wifiUp = true;
}

// Send one beacon whose only IE is our vendor IE wrapping `body` (tag+payload).
// Returns ESP-IDF error code (0 = OK).
static int sendBeacon(const uint8_t *body, size_t bodyLen) {
  if (bodyLen > 70) return -1;
  wifiTxInit();
  esp_wifi_set_channel(txChannel, WIFI_SECOND_CHAN_NONE);

  uint8_t mac[6];
  esp_wifi_get_mac(g_txIf, mac);

  uint8_t f[128];
  size_t o = 0;
  f[o++] = 0x80; f[o++] = 0x00;                   // FC: beacon
  f[o++] = 0x00; f[o++] = 0x00;                   // duration
  for (int i = 0; i < 6; i++) f[o++] = 0xFF;      // DA broadcast
  for (int i = 0; i < 6; i++) f[o++] = mac[i];    // SA
  for (int i = 0; i < 6; i++) f[o++] = mac[i];    // BSSID
  f[o++] = 0x00; f[o++] = 0x00;                   // seq (filled by en_sys_seq)
  for (int i = 0; i < 8; i++) f[o++] = 0x00;      // timestamp
  f[o++] = 0x64; f[o++] = 0x00;                   // interval 100 TU
  f[o++] = 0x21; f[o++] = 0x00;                   // capability
  f[o++] = 0xDD;                                  // vendor IE
  f[o++] = (uint8_t)(3 + bodyLen);
  f[o++] = 0x52; f[o++] = 0x43; f[o++] = 0x68;    // OUI "RCh"
  memcpy(f + o, body, bodyLen);
  o += bodyLen;

  return esp_wifi_80211_tx(g_txIf, f, o, true);
}

static int sendBitfield(uint8_t tag, const uint8_t *bits) {
  uint8_t body[33];
  body[0] = tag;
  memcpy(body + 1, bits, 32);
  return sendBeacon(body, 33);
}

// Parse "42:1,7:0,..." into a bitfield (updates only the listed ids).
static bool applyBitSpec(uint8_t *bits, const String &spec) {
  int pos = 0;
  while (pos < (int)spec.length()) {
    int colon = spec.indexOf(':', pos);
    if (colon < 0) return false;
    int comma = spec.indexOf(',', colon);
    if (comma < 0) comma = spec.length();
    int id = spec.substring(pos, colon).toInt();
    int val = spec.substring(colon + 1, comma).toInt();
    if (id < 0 || id > 255) return false;
    if (val) bits[id / 8] |= (1u << (id % 8));
    else bits[id / 8] &= ~(1u << (id % 8));
    pos = comma + 1;
  }
  return true;
}

static void cenPulse(uint32_t ms) {
  digitalWrite(PIN_CEN, LOW);
  delay(ms);
  digitalWrite(PIN_CEN, HIGH);
}

// Exit-bridge magic (PC->BK stream): restart ESP back into command mode
static const uint8_t BRIDGE_EXIT[] = {0xDE, 0xAD, 0xBE, 0xEF, 'B', 'K', 'J', 'I', 'G', '-', 'E', 'X', 'I', 'T', '4', '2'};

// BK7231-protocol BootROM link-check RESPONSE preamble (PACKET_RESP_PREAMBLE).
// Seen on the BK->PC stream once the BootROM answers the flasher's LinkCheck.
static const uint8_t BOOTROM_LINK[] = {0x04, 0x0E};

// CEN pulsing must NEVER overlap the actual flash. The flasher links within a
// second or two of a reset, then spends tens of seconds erasing/writing; a CEN
// pulse landing mid-erase corrupts flash (this is what was bricking the
// bootloader). So pulse only during a short startup WINDOW: a few resets to
// catch the BootROM while the flasher polls, then stop for good. If the catch
// is missed the flasher just errors and we retry — no corruption. Detecting
// the link response stops pulsing even earlier, but the window is the guarantee.
#define BOOTROM_PULSE_WINDOW_MS 9000u
#define BOOTROM_PULSE_PERIOD_MS 2500u

// Transparent bridge. bootromCatch: pulse CEN to catch the BootROM, bounded by
// the startup window above.
static void bridgeLoop(bool bootromCatch) {
  uint32_t t0 = millis();
  uint32_t nextPulse = t0 + 800;
  bool linked = !bootromCatch;
  size_t linkMatch = 0;
  size_t exitMatch = 0;
  uint8_t buf[512];
  for (;;) {
    if (!linked && millis() - t0 < BOOTROM_PULSE_WINDOW_MS &&
        (int32_t)(millis() - nextPulse) >= 0) {
      digitalWrite(PIN_CEN, LOW);
      delay(60);
      digitalWrite(PIN_CEN, HIGH);
      nextPulse = millis() + BOOTROM_PULSE_PERIOD_MS;
    }
    int n = Serial.available();
    if (n > 0) {
      n = Serial.read(buf, (size_t)min(n, 512));
      if (n > 0) {
        for (int i = 0; i < n; i++) {
          exitMatch = (buf[i] == BRIDGE_EXIT[exitMatch]) ? exitMatch + 1
                    : (buf[i] == BRIDGE_EXIT[0]) ? 1 : 0;
          if (exitMatch == sizeof(BRIDGE_EXIT)) {
            prefs.putUChar("fm", 0);
            delay(20);
            ESP.restart();
          }
        }
        Serial1.write(buf, n);
      }
    }
    n = Serial1.available();
    if (n > 0) {
      n = Serial1.read(buf, (size_t)min(n, 512));
      if (n > 0) {
        if (!linked) {
          for (int i = 0; i < n; i++) {
            linkMatch = (buf[i] == BOOTROM_LINK[linkMatch]) ? linkMatch + 1
                      : (buf[i] == BOOTROM_LINK[0]) ? 1 : 0;
            if (linkMatch == sizeof(BOOTROM_LINK)) linked = true;
          }
        }
        Serial.write(buf, n);
      }
    }
  }
}

static int hexNib(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static void printState() {
  Serial.printf("LED=%d SW=%d BAUD=%lu UPTIME=%lu\r\n",
                digitalRead(PIN_LED), digitalRead(PIN_SW),
                (unsigned long)bkBaud, (unsigned long)millis());
}

// Stream BK UART + pin-change events until any byte arrives from PC
static void watchLoop(bool hexDump) {
  int led = digitalRead(PIN_LED), sw = digitalRead(PIN_SW);
  Serial.printf("WATCH START LED=%d SW=%d\r\n", led, sw);
  uint8_t buf[256];
  for (;;) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.println("WATCH END");
      return;
    }
    int l = digitalRead(PIN_LED);
    if (l != led) { led = l; Serial.printf("EVT LED %d t=%lu\r\n", l, (unsigned long)millis()); }
    int s = digitalRead(PIN_SW);
    if (s != sw) { sw = s; Serial.printf("EVT SW %d t=%lu\r\n", s, (unsigned long)millis()); }
    int n = Serial1.available();
    if (n > 0) {
      n = Serial1.read(buf, (size_t)min(n, 256));
      if (hexDump) {
        Serial.printf("RX[%d]", n);
        for (int i = 0; i < n; i++) Serial.printf(" %02X", buf[i]);
        Serial.println();
      } else {
        Serial.write(buf, n);
      }
    }
  }
}

static void handleLine(String line) {
  line.trim();
  if (line.length() == 0) return;
  String cmd = line;
  String arg = "";
  int sp = line.indexOf(' ');
  if (sp >= 0) { cmd = line.substring(0, sp); arg = line.substring(sp + 1); arg.trim(); }
  cmd.toUpperCase();

  if (cmd == "PING") {
    Serial.println(VERSION " PONG");
  } else if (cmd == "STATE") {
    printState();
  } else if (cmd == "LED?") {
    Serial.println(digitalRead(PIN_LED));
  } else if (cmd == "SW?") {
    Serial.println(digitalRead(PIN_SW));
  } else if (cmd == "BTN") {
    arg.toUpperCase();
    if (arg == "DOWN") { digitalWrite(PIN_BTN, LOW); Serial.println("BTN DOWN OK"); }
    else if (arg == "UP") { digitalWrite(PIN_BTN, HIGH); Serial.println("BTN UP OK"); }
    else {
      long ms = arg.startsWith("PRESS") ? arg.substring(5).toInt() : 0;
      if (ms <= 0) ms = 200;
      digitalWrite(PIN_BTN, LOW);
      delay(ms);
      digitalWrite(PIN_BTN, HIGH);
      Serial.printf("BTN PRESSED %ldms OK\r\n", ms);
    }
  } else if (cmd == "CEN") {
    arg.toUpperCase();
    if (arg == "LOW") { digitalWrite(PIN_CEN, LOW); Serial.println("CEN LOW OK"); }
    else if (arg == "HIGH") { digitalWrite(PIN_CEN, HIGH); Serial.println("CEN HIGH OK"); }
    else {
      long ms = arg.startsWith("PULSE") ? arg.substring(5).toInt() : 0;
      if (ms <= 0) ms = 100;
      cenPulse(ms);
      Serial.printf("CEN PULSED %ldms OK\r\n", ms);
    }
  } else if (cmd == "REBOOT") {
    cenPulse(100);
    Serial.println("BK REBOOTED OK");
  } else if (cmd == "BAUD") {
    long b = arg.toInt();
    if (b >= 300 && b <= 3000000) {
      bkBaud = b;
      Serial1.updateBaudRate(bkBaud);
      Serial.printf("BAUD %lu OK\r\n", (unsigned long)bkBaud);
    } else {
      Serial.println("ERR bad baud");
    }
  } else if (cmd == "SEND") {
    // SEND <hex bytes, spaces optional>
    uint8_t out[256];
    int n = 0, hi = -1;
    for (size_t i = 0; i < arg.length() && n < 256; i++) {
      int v = hexNib(arg[i]);
      if (v < 0) continue;
      if (hi < 0) hi = v;
      else { out[n++] = (hi << 4) | v; hi = -1; }
    }
    Serial1.write(out, n);
    Serial.printf("SENT %d OK\r\n", n);
  } else if (cmd == "MON") {
    watchLoop(false);
  } else if (cmd == "WATCH") {
    watchLoop(true);
  } else if (cmd == "BRIDGE") {
    Serial.println("BRIDGE OK");
    Serial.flush();
    bridgeLoop(false);
  } else if (cmd == "FLASHMODE") {
    // NVS flag covers the case where the ESP resets when the port reopens;
    // otherwise the bridge entered below just survives the reopen.
    prefs.putUChar("fm", 1);
    Serial.println("FLASHMODE OK ENTERING BRIDGE");
    Serial.flush();
    bridgeLoop(true);  // never returns (exit magic restarts the ESP)
  } else if (cmd == "PASS") {
    arg.toUpperCase();
    g_passthrough = (arg == "ON" || arg.length() == 0);
    Serial.printf("PASS %s OK\r\n", g_passthrough ? "ON" : "OFF");
  } else if (cmd == "DFULOAD") {
    // DFULOAD <size> <sha256hex> <buildidhex> ; then stream <size> raw bytes
    long sz = 0; String shaHex, bidHex;
    int a1 = arg.indexOf(' '); int a2 = arg.indexOf(' ', a1 + 1);
    if (a1 < 0 || a2 < 0) { Serial.println("ERR DFULOAD <size> <sha> <buildid>"); return; }
    sz = arg.substring(0, a1).toInt();
    shaHex = arg.substring(a1 + 1, a2); shaHex.trim();
    bidHex = arg.substring(a2 + 1); bidHex.trim();
    if (g_rbl) { free(g_rbl); g_rbl = NULL; }
    g_rbl = (uint8_t *)ps_malloc(sz);
    if (!g_rbl) { Serial.println("ERR ps_malloc failed"); return; }
    // parse sha + build id
    for (int i = 0; i < 32; i++) g_rblSha[i] = (hexNib(shaHex[2*i]) << 4) | hexNib(shaHex[2*i+1]);
    for (int i = 0; i < 8; i++)  g_dfuBuildId[i] = (hexNib(bidHex[2*i]) << 4) | hexNib(bidHex[2*i+1]);
    g_rblSize = sz;
    Serial.printf("DFULOAD ready for %ld bytes, send now\r\n", sz);
    Serial.flush();
    // read sz raw bytes from serial
    uint32_t got = 0; uint32_t t0 = millis();
    while (got < (uint32_t)sz && millis() - t0 < 120000) {
      int n = Serial.available();
      if (n > 0) { size_t want = (size_t)((uint32_t)sz - got); if ((size_t)n < want) want = (size_t)n; n = Serial.read(g_rbl + got, want); got += n; t0 = millis(); }
    }
    Serial.printf("DFULOAD got %lu/%ld %s\r\n", got, sz, got == (uint32_t)sz ? "OK" : "TIMEOUT");
  } else if (cmd == "DFUAP") {
    // start softAP "RC-Update" on the switch's channel + TCP server on 3333
    WiFi.mode(WIFI_AP);
    wifiUp = true;
    g_txIf = WIFI_IF_AP;
    WiFi.softAP("RC-Update", "", txChannel);
    g_dfuServer.begin();
    g_dfuServerActive = true;
    Serial.printf("DFUAP up: SSID RC-Update ch%d ip=%s server:3333\r\n",
                  txChannel, WiFi.softAPIP().toString().c_str());
  } else if (cmd == "DFU2BC") {
    // broadcast a 0x05 Dfu2Request: device_type=relay, id range, server=AP ip, RC-Update
    int id = arg.length() ? arg.toInt() : 5;
    IPAddress ip = WiFi.softAPIP();  // 192.168.4.1
    uint8_t body[80]; int o = 0;
    body[o++] = 0x05;            // tag
    body[o++] = 0x02;            // device_type = relay
    body[o++] = (uint8_t)id;     // id_start
    body[o++] = 0x00;            // id_bounds
    body[o++] = ip[0]; body[o++] = ip[1]; body[o++] = ip[2]; body[o++] = ip[3];
    body[o++] = 3333 & 0xFF; body[o++] = (3333 >> 8) & 0xFF;   // port LE
    body[o++] = 8;               // build_id_len
    for (int i = 0; i < 8; i++) body[o++] = g_dfuBuildId[i];
    const char *ss = "RC-Update";
    body[o++] = 9; for (int i = 0; i < 9; i++) body[o++] = ss[i];  // ssid
    body[o++] = 0;               // pass_len (open)
    int err = sendBeacon(body, o);
    Serial.printf("DFU2BC id=%d ip=%s err=%d %s\r\n", id, ip.toString().c_str(), err, err == 0 ? "OK" : "FAIL");
  } else if (cmd == "NET") {
    // NET <ssid> [pass] — join an AP as STA (for talking to OpenBeken's web API)
    String ssid = arg, pass = "";
    int sp2 = arg.indexOf(' ');
    if (sp2 >= 0) { ssid = arg.substring(0, sp2); pass = arg.substring(sp2 + 1); ssid.trim(); pass.trim(); }
    WiFi.mode(WIFI_STA);
    wifiUp = true;
    if (pass.length()) WiFi.begin(ssid.c_str(), pass.c_str());
    else WiFi.begin(ssid.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) delay(200);
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("NET OK ip=%s\r\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("NET FAIL");
    }
  } else if (cmd == "GET") {
    // GET <path> — HTTP GET http://192.168.4.1<path>, print body (OpenBeken /cm API)
    if (WiFi.status() != WL_CONNECTED) { Serial.println("ERR not connected (NET first)"); return; }
    HTTPClient http;
    String url = "http://192.168.4.1" + (arg.startsWith("/") ? arg : "/" + arg);
    http.begin(url);
    int code = http.GET();
    Serial.printf("GET %d\r\n", code);
    if (code > 0) {
      String body = http.getString();
      Serial.println(body);   // full body (log reads can be large)
    }
    http.end();
    Serial.println("GET DONE");
  } else if (cmd == "SCAN") {
    // Quick WiFi scan — used to detect whether the BK booted OpenBeken (which
    // brings up an "OpenBK7238_*"/"OBK*" config AP when unconfigured).
    WiFi.mode(WIFI_STA);
    wifiUp = true;
    int n = WiFi.scanNetworks(false, true);
    Serial.printf("SCAN found %d\r\n", n);
    for (int i = 0; i < n; i++) {
      Serial.printf("  %s ch%d %ddBm\r\n", WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i));
    }
    WiFi.scanDelete();
    Serial.println("SCAN DONE");
  } else if (cmd == "CHAN") {
    long ch = arg.toInt();
    if (ch >= 1 && ch <= 13) {
      txChannel = (uint8_t)ch;
      if (wifiUp) esp_wifi_set_channel(txChannel, WIFI_SECOND_CHAN_NONE);
      Serial.printf("CHAN %d OK\r\n", (int)ch);
    } else {
      Serial.println("ERR bad channel");
    }
  } else if (cmd == "RELAYS" || cmd == "RELAYSLED") {
    uint8_t *bits = (cmd == "RELAYS") ? relayBits : ledBits;
    uint8_t tag = (cmd == "RELAYS") ? 0x40 : 0x41;
    if (arg.length() && !applyBitSpec(bits, arg)) {
      Serial.println("ERR bad bitspec (want id:val,id:val,...)");
      return;
    }
    int err = sendBitfield(tag, bits);
    Serial.printf("%s TX ch%d err=%d %s\r\n", cmd.c_str(), txChannel, err, err == 0 ? "OK" : "FAIL");
  } else if (cmd == "BCAST") {
    // BCAST <hex body: tag+payload>
    uint8_t body[70];
    int n = 0, hi = -1;
    for (size_t i = 0; i < arg.length() && n < 70; i++) {
      int v = hexNib(arg[i]);
      if (v < 0) continue;
      if (hi < 0) hi = v;
      else { body[n++] = (hi << 4) | v; hi = -1; }
    }
    if (n < 1) { Serial.println("ERR empty body"); return; }
    int err = sendBeacon(body, n);
    Serial.printf("BCAST %dB ch%d err=%d %s\r\n", n, txChannel, err, err == 0 ? "OK" : "FAIL");
  } else if (cmd == "AUTO") {
    arg.toUpperCase();
    if (arg == "OFF") { autoPeriodMs = 0; Serial.println("AUTO OFF OK"); }
    else {
      long ms = arg.startsWith("ON") ? arg.substring(2).toInt() : arg.toInt();
      if (ms < 50) ms = 500;
      autoPeriodMs = ms;
      autoLastMs = 0;
      Serial.printf("AUTO %ldms OK (broadcasting RELAYS+RELAYSLED)\r\n", ms);
    }
  } else if (cmd == "RESTART") {
    Serial.println("RESTART OK");
    Serial.flush();
    ESP.restart();
  } else if (cmd == "HELP") {
    Serial.println("PING STATE LED? SW? BTN[DOWN|UP|PRESS ms] CEN[LOW|HIGH|PULSE ms] REBOOT BAUD n SEND hex MON WATCH BRIDGE FLASHMODE RESTART CHAN n RELAYS id:v,... RELAYSLED id:v,... BCAST hex AUTO [ms|OFF]");
  } else {
    Serial.println("ERR unknown cmd (try HELP)");
  }
}

void setup() {
  // Open-drain outputs: preload released level before enabling the pin
  digitalWrite(PIN_CEN, HIGH);
  pinMode(PIN_CEN, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_CEN, HIGH);
  digitalWrite(PIN_BTN, HIGH);
  pinMode(PIN_BTN, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_BTN, HIGH);
  pinMode(PIN_LED, INPUT);
  pinMode(PIN_SW, INPUT);

  Serial.setRxBufferSize(8192);
  Serial.begin(115200);
  Serial1.setRxBufferSize(8192);
  Serial1.begin(bkBaud, SERIAL_8N1, PIN_RX_FROM_BK, PIN_TX_TO_BK);

  prefs.begin("jig", false);
  if (prefs.getUChar("fm", 0) == 1) {
    prefs.putUChar("fm", 0);
    bridgeLoop(true);  // never returns
  }

  Serial.println();
  Serial.println(VERSION " READY");
  printState();
}

String lineBuf;

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineBuf.length()) { handleLine(lineBuf); lineBuf = ""; }
    } else if (lineBuf.length() < 300) {
      lineBuf += c;
    }
  }
  if (autoPeriodMs && millis() - autoLastMs >= autoPeriodMs) {
    autoLastMs = millis();
    sendBitfield(0x40, relayBits);
    sendBitfield(0x41, ledBits);
  }

  // Passthrough: relay BK UART1 (switch logs) to USB so they're visible in tests.
  if (g_passthrough) {
    while (Serial1.available()) Serial.write(Serial1.read());
  }

  // DFU2 test server: serve the header + RBL to a connecting switch.
  if (g_dfuServerActive && g_rbl && g_rblSize) {
    WiFiClient c = g_dfuServer.available();
    if (c) {
      Serial.println("DFU2: client connected");
      uint8_t hello[14];
      uint32_t t0 = millis(); int got = 0;
      while (got < 14 && millis() - t0 < 5000) {
        if (c.available()) hello[got++] = c.read();
      }
      Serial.printf("DFU2: hello %d bytes (id=%d)\r\n", got, got >= 6 ? hello[5] : -1);
      // header: magic("RClh" LE) | ver | image_len LE | sha256[32] | build_id[8]
      uint8_t hdr[49]; int o = 0;
      hdr[o++] = 0x52; hdr[o++] = 0x43; hdr[o++] = 0x6C; hdr[o++] = 0x68;  // "RClh" = 0x686C4352
      hdr[o++] = 0x01;
      hdr[o++] = g_rblSize & 0xFF; hdr[o++] = (g_rblSize >> 8) & 0xFF;
      hdr[o++] = (g_rblSize >> 16) & 0xFF; hdr[o++] = (g_rblSize >> 24) & 0xFF;
      memcpy(hdr + o, g_rblSha, 32); o += 32;
      memcpy(hdr + o, g_dfuBuildId, 8); o += 8;
      c.write(hdr, 49);
      // stream the RBL
      uint32_t sent = 0;
      while (sent < g_rblSize && c.connected()) {
        uint32_t chunk = min((uint32_t)2048, g_rblSize - sent);
        int w = c.write(g_rbl + sent, chunk);
        if (w <= 0) break;
        sent += w;
      }
      Serial.printf("DFU2: sent %lu/%lu bytes\r\n", sent, g_rblSize);
      // read the 1-byte status reply
      t0 = millis(); int st = -1;
      while (millis() - t0 < 5000) { if (c.available()) { st = c.read(); break; } }
      Serial.printf("DFU2: switch status=%d %s\r\n", st, st == 0 ? "(OK, rebooting)" : "");
      c.stop();
    }
  }
  delay(1);
}
