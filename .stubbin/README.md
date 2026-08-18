# .stubbin — Windows build workaround

ESP8266_RTOS_SDK's CMake `__kconfig_init` refuses to configure on Windows unless
a `mconf-idf` (Kconfig menuconfig tool) executable is found on `PATH`, OR a native
host `gcc` is available to build it. This machine has only the xtensa **cross**
compiler, so neither is present.

`mconf-idf` is only ever invoked by the interactive `menuconfig` target. A normal
build generates its config with the pure-Python `confgen.py`, so a stub is enough
to get past the check.

`mconf-idf.exe` here is a trivial stub (`mconf_stub.c`: `int main(void){return 1;}`).
Put this directory first on `PATH` when building (see `../build.sh`). Do **not**
run `menuconfig` with this stub on PATH — it does nothing useful.

Rebuild the stub if missing (needs MSVC `cl.exe`; see the host-C-toolchain notes):
    cl /nologo mconf_stub.c /Fe:mconf-idf.exe
