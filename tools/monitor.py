#!/usr/bin/env python3
"""Reset an ESP8266 into run mode and print its serial console.

The firmware console runs at 74880 baud (CONFIG_ESP_CONSOLE_UART_BAUDRATE).

    python3 tools/monitor.py [PORT] [BAUD] [SECONDS]
    python3 tools/monitor.py COM8            # 74880 baud, run until Ctrl-C
"""
import serial, sys, time

port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 74880
secs = float(sys.argv[3]) if len(sys.argv) > 3 else None

s = serial.Serial(port, baud, timeout=0.2)
# Auto-reset into run mode: DTR->GPIO0 (high=normal boot), RTS->RST (pulse low).
s.setDTR(False)
s.setRTS(True); time.sleep(0.1)
s.setRTS(False)

end = time.time() + secs if secs else None
try:
    while end is None or time.time() < end:
        data = s.read(4096)
        if data:
            sys.stdout.write(data.decode("utf-8", "replace"))
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    s.close()
