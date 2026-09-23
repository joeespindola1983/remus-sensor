FILE=$(find .pio -name "downloaded_fs_*.bin" | head -1)

python3 - "$FILE" <<'PY'
import sys

p = sys.argv[1]
data = open(p, "rb").read()

ff = data.count(0xff)
zero = data.count(0x00)

print("File:", p)
print("Size:", len(data), "bytes")
print("0xFF:", f"{ff / len(data) * 100:.2f}%")
print("0x00:", f"{zero / len(data) * 100:.2f}%")
print("Other:", f"{(len(data)-ff-zero) / len(data) * 100:.2f}%")
PY
