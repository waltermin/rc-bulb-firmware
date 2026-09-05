#!/usr/bin/env python3
"""Build and run the host unit tests with whatever C compiler is available
(cc/gcc/clang, else `python -m ziglang cc`)."""
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).parent
SRC = [HERE / "test_protocol.c", HERE.parent / "main" / "protocol.c",
       HERE.parent / "main" / "controller.c"]
OUT = HERE / ("test_protocol.exe" if sys.platform == "win32" else "test_protocol")

FLAGS = ["-std=c11", "-Wall", "-Wextra", "-Werror", "-O1"]


def compiler():
    for cand in ("cc", "gcc", "clang"):
        if shutil.which(cand):
            return [cand]
    try:
        import ziglang  # noqa: F401
        return [sys.executable, "-m", "ziglang", "cc"]
    except ImportError:
        sys.exit("no C compiler found (install gcc/clang or `pip install ziglang`)")


def main():
    cmd = compiler() + FLAGS + [str(s) for s in SRC] + ["-o", str(OUT)]
    print(" ".join(cmd))
    subprocess.run(cmd, check=True)
    result = subprocess.run([str(OUT)])
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
