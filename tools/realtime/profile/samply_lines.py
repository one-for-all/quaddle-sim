"""Self time per source line from a `samply record --save-only` profile.

Usage: python3 samply_lines.py profile.json.gz path/to/binary [addresses] [top N]
Run `dsymutil path/to/binary` first so atos can map addresses to lines.
"""
import gzip, json, sys, subprocess, collections, re
prof, binary = sys.argv[1], sys.argv[2]
top = int(sys.argv[3]) if len(sys.argv) > 3 else 400
p = json.load(gzip.open(prof))
t = p["threads"][0]
ft, st, rt = t["frameTable"], t["stackTable"], t["resourceTable"]
libidx = next(i for i, l in enumerate(p["libs"]) if l["path"] == binary)
cnt = collections.Counter(); total = 0
for s, w in zip(t["samples"]["stack"], t["samples"].get("weight") or [1]*t["samples"]["length"]):
    if s is None: continue
    total += (w or 1)
    f = st["frame"][s]
    func = ft["func"][f]
    res = t["funcTable"]["resource"][func]
    if res is None or res < 0 or rt["lib"][res] != libidx: continue
    cnt[ft["address"][f]] += (w or 1)
addrs = [a for a, _ in cnt.most_common(top)]
out = subprocess.run(["atos", "-o", binary, "-l", "0x100000000", "-i"] + [hex(a + 0x100000000) for a in addrs], capture_output=True, text=True).stdout
# atos -i prints inlined chains separated by blank lines
blocks = out.strip().split("\n\n")
byline = collections.Counter(); byfunc = collections.Counter()
for a, blk in zip(addrs, blocks):
    first = blk.split("\n")[0]
    m = re.search(r"\(in [^)]*\) \((.*?)\)$", first)
    loc = m.group(1) if m else "?"
    fn = re.sub(r" \(in [^)]*\).*$", "", first)
    fn = re.sub(r"::h[0-9a-f]{16}$", "", fn)
    byline[(fn[-70:], loc)] += cnt[a]
for (fn, loc), c in byline.most_common(int(sys.argv[4]) if len(sys.argv) > 4 else 50):
    print(f"{100*c/total:5.1f}% {loc:40} {fn}")
