"""Self time per function (with source line) from a `samply record --save-only` profile.

Usage: python3 samply_funcs.py profile.json.gz path/to/binary [top N]
Run `dsymutil path/to/binary` first so atos can map addresses to lines.
"""
import gzip, json, sys, subprocess, collections, re
prof, binary = sys.argv[1], sys.argv[2]
p = json.load(gzip.open(prof))
t = p["threads"][0]
ft, st, rt = t["frameTable"], t["stackTable"], t["resourceTable"]
libidx = next(i for i, l in enumerate(p["libs"]) if l["path"] == binary)
cnt = collections.Counter(); total = 0
for s in t["samples"]["stack"]:
    if s is None: continue
    total += 1
    f = st["frame"][s]; func = ft["func"][f]; res = t["funcTable"]["resource"][func]
    if res is None or res < 0 or rt["lib"][res] != libidx: cnt[-1] += 1; continue
    cnt[ft["address"][f]] += 1
addrs = [a for a in cnt if a >= 0]
out = subprocess.run(["atos", "-o", binary, "-l", "0x100000000"] + [hex(a + 0x100000000) for a in addrs], capture_output=True, text=True).stdout.strip().split("\n")
byfn = collections.Counter()
for a, line in zip(addrs, out):
    fn = re.sub(r" \(in [^)]*\).*$", "", line); fn = re.sub(r"::h[0-9a-f]{16}$", "", fn)
    byfn[fn] += cnt[a]
byfn["(other libs)"] = cnt[-1]
for fn, c in byfn.most_common(int(sys.argv[3]) if len(sys.argv) > 3 else 40):
    print(f"{100*c/total:5.1f}% {fn[:140]}")
