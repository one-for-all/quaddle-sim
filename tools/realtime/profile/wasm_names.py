"""Dump the function-name section of a .wasm file (index -> name).

Usage: python3 wasm_names.py module.wasm
"""
import sys
def leb(b, i):
    r = s = 0
    while True:
        x = b[i]; i += 1; r |= (x & 0x7f) << s; s += 7
        if x < 0x80: return r, i
b = open(sys.argv[1], "rb").read()
i = 8; names = {}; nimports = 0
while i < len(b):
    sid = b[i]; i += 1
    size, i = leb(b, i); end = i + size
    if sid == 2:  # imports: count function imports
        n, j = leb(b, i)
        for _ in range(n):
            l, j = leb(b, j); j += l
            l, j = leb(b, j); j += l
            kind = b[j]; j += 1
            if kind == 0: _, j = leb(b, j); nimports += 1
            elif kind == 1: j += 1; fl, j = leb(b, j); _, j = leb(b, j); 
            elif kind == 2:
                fl, j = leb(b, j); _, j = leb(b, j)
                if fl & 1: _, j = leb(b, j)
            elif kind == 3: j += 2
    if sid == 0:
        l, j = leb(b, i); nm = b[j:j+l].decode(); j += l
        if nm == "name":
            while j < end:
                sub = b[j]; j += 1; ss, j = leb(b, j); se = j + ss
                if sub == 1:
                    n, j = leb(b, j)
                    for _ in range(n):
                        idx, j = leb(b, j); l, j = leb(b, j); names[idx] = b[j:j+l].decode(errors="replace"); j += l
                j = se
    i = end
print("imports", nimports, "names", len(names), file=sys.stderr)
for k, v in names.items(): print(k, v)
