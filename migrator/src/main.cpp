// Trampoline migrator: booted by the stock eboot at 0x1000 after the ESPHome
// DFU upload. Validates the pre-positioned RTOS payload (CRC-32), then hands off
// to the IRAM flash-rewrite engine, which lays the RTOS full-flash image at 0x0
// and resets into the new firmware. See README.md.
#include <Arduino.h>
#include <spi_flash.h>
#include "rom_funcs.h"
#include "payload_hdr.h"
#include "flash_rewrite.h"
#include "leds.h"

// Streaming zlib/IEEE CRC-32 (poly 0xEDB88320). Feed chunks; init crc=0xFFFFFFFF,
// finalize with ^0xFFFFFFFF. Matches Python zlib.crc32.
static uint32_t crc32_stream(uint32_t crc, const uint8_t *data, uint32_t len) {
  for (uint32_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int k = 0; k < 8; k++) {
      crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
  }
  return crc;
}

// Cache-on flash read (full-chip correct). dst 4-aligned, len rounded up to 4.
static int flash_read(uint32_t addr, void *dst, uint32_t len) {
  return spi_flash_read(addr, (uint32_t *)dst, (len + 3u) & ~3u);
}

// One-shot diagnostic: show what ROM SPIRead vs SDK spi_flash_read return at addr.
static void diag_read(uint32_t addr) {
  uint8_t a[8] = {0}, b[8] = {0};
  SPIRead(addr, a, 8);            // ROM (cache on)
  spi_flash_read(addr, (uint32_t *)b, 8);  // SDK
  Serial.printf("[migrator] @0x%06X  SPIRead=%02x%02x%02x%02x%02x%02x%02x%02x  "
                "spi_flash_read=%02x%02x%02x%02x%02x%02x%02x%02x\n", addr,
                a[0],a[1],a[2],a[3],a[4],a[5],a[6],a[7],
                b[0],b[1],b[2],b[3],b[4],b[5],b[6],b[7]);
}

// Validation failed before any destructive write: flash red forever (cache on,
// so plain analogWrite; capped at LED_MAX). Recover via serial.
static void fail(const char *msg) {
  Serial.print(F("[migrator] FAIL: "));
  Serial.println(msg);
  Serial.flush();
  analogWrite(LED_B_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  for (;;) {
    analogWrite(LED_R_PIN, LED_MAX); delay(180);
    analogWrite(LED_R_PIN, 0);       delay(180);
  }
}

void setup() {
  Serial.begin(74880);
  delay(50);
  Serial.println(F("\n[migrator] KMIG trampoline start"));

  // Solid blue = migrating. analogWrite drives the Timer1 NMI (IRAM,
  // non-maskable), so this color persists through the cache-off flash write.
  analogWriteRange(255);
  pinMode(LED_R_PIN, OUTPUT); pinMode(LED_G_PIN, OUTPUT); pinMode(LED_B_PIN, OUTPUT);
  analogWrite(LED_R_PIN, 0);
  analogWrite(LED_G_PIN, 0);
  analogWrite(LED_B_PIN, LED_MAX);

  // Diagnostic: ROM SPIRead mis-reads >1 MB with the cache on; SDK read is correct.
  diag_read(0x000000);   // migrator eboot header (e9 02 03 40)
  diag_read(0x110000);   // KMIG header (4b 4d 49 47 ...)
  diag_read(0x111000);   // RTOS bootloader (e9 03 02 40)

  payload_hdr_t h;
  if (flash_read(MIG_HDR_ADDR, &h, sizeof(h))) fail("flash_read header");

  if (h.magic != MIG_MAGIC)   fail("bad magic");
  uint32_t hdr_crc = crc32_stream(0xFFFFFFFFu, (const uint8_t *)&h, MIG_HDR_CRC_LEN) ^ 0xFFFFFFFFu;
  if (hdr_crc != h.hdr_crc32) fail("bad header CRC");
  if (h.version != MIG_VERSION) fail("bad version");
  if (h.target_base != 0)     fail("target_base != 0");
  if (h.length == 0 || h.length > 0x100000u) fail("bad length");
  if (h.sector_count != (h.length + 4095u) / 4096u) fail("sector_count mismatch");

  Serial.print(F("[migrator] payload len=0x")); Serial.print(h.length, HEX);
  Serial.print(F(" sectors="));                 Serial.println(h.sector_count);

  // Validate the whole payload before touching anything destructive.
  uint32_t crc = 0xFFFFFFFFu;
  static uint8_t buf[4096];
  for (uint32_t off = 0; off < h.length; off += sizeof(buf)) {
    uint32_t n = h.length - off;
    if (n > sizeof(buf)) n = sizeof(buf);
    if (flash_read(MIG_PAYLOAD_ADDR + off, buf, n)) fail("flash_read payload");
    crc = crc32_stream(crc, buf, n);
  }
  crc ^= 0xFFFFFFFFu;
  if (crc != h.payload_crc32) fail("payload CRC mismatch");

  Serial.println(F("[migrator] payload CRC OK -> migrating (do not power off)"));
  Serial.flush();

  migrator_do_migration(MIG_PAYLOAD_ADDR, h.sector_count);  // never returns
}

void loop() {}
