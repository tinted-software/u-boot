#!/usr/bin/env bash
# Package the freshly built U-Boot as an Android boot image and boot it
# (non-destructively) on the connected Pixel 7a over fastboot.
#
# ABL on gs201 enforces AVB even for `fastboot boot`, so mirror the trick
# used by the Linux bring-up script: append an empty vbmeta image plus a
# matching AVB footer so the image self-validates.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

OUT="$ROOT/lynx-u-boot.img"
EMPTY="$(mktemp)"
trap 'rm -f "$EMPTY"' EXIT

mkbootimg \
  --kernel "$ROOT/u-boot.bin" \
  --ramdisk "$EMPTY" \
  --header_version 4 \
  --pagesize 4096 \
  --cmdline "console=ttySAC0,115200n8 androidboot.hardware=gs201 androidboot.serialconsole=1" \
  --output "$OUT"

python3 - "$OUT" <<'PY'
import sys
from pathlib import Path

p = Path(sys.argv[1])
img = p.read_bytes()
avbf = img.rfind(b"AVBf")
if avbf != -1 and avbf >= len(img) - 64:
    orig = int.from_bytes(img[avbf + 12:avbf + 20], "big")
    if 0 < orig <= avbf:
        img = img[:orig]

vbmeta = bytearray(256)
vbmeta[0:4] = b"AVB0"
vbmeta[4:8] = (1).to_bytes(4, "big")
n = len(img)
footer = bytearray(64)
footer[0:4] = b"AVBf"
footer[4:8] = (1).to_bytes(4, "big")
footer[12:20] = n.to_bytes(8, "big")
footer[20:28] = n.to_bytes(8, "big")
footer[28:36] = (256).to_bytes(8, "big")
p.write_bytes(img + bytes(vbmeta) + bytes(footer))
print("wrote", p, "bytes", p.stat().st_size)
PY

fastboot boot "$OUT"
