"""Self or inclusive time per function from `node --cpu-prof` output (wasm included).

Usage: python3 node_cpuprof.py file.cpuprofile [self|incl] [top N]
Needs a wasm build that keeps function names: `wasm-pack build --profiling` with
`wasm-opt = ["-O", "-g"]` (see ../wasm/Cargo.toml).
"""
import json, sys, collections, re
p = json.load(open(sys.argv[1]))
nodes = {n["id"]: n for n in p["nodes"]}
parent = {}
for n in p["nodes"]:
    for c in n.get("children", []):
        parent[c] = n["id"]
selfc = collections.Counter()
for s in p["samples"]:
    selfc[s] += 1
total = sum(selfc.values())
byname = collections.Counter()
incl = collections.Counter()
for nid, c in selfc.items():
    name = nodes[nid]["callFrame"]["functionName"] or "(anon)"
    byname[name] += c
    seen = set(); x = nid
    while x is not None:
        nm = nodes[x]["callFrame"]["functionName"] or "(anon)"
        if nm not in seen:
            incl[nm] += c; seen.add(nm)
        x = parent.get(x)
mode = sys.argv[2] if len(sys.argv) > 2 else "self"
src = byname if mode == "self" else incl
for name, c in src.most_common(int(sys.argv[3]) if len(sys.argv) > 3 else 50):
    print(f"{100*c/total:5.1f}% {name[:140]}")
