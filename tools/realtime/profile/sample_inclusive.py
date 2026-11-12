"""Inclusive time per function from a macOS `sample` report.

Usage: python3 sample_inclusive.py report.txt [regex filter] [top N]
(`sample <pid> 10 -file report.txt` while a native bench runs.)
"""
import re, sys, collections
path = sys.argv[1]
pat = sys.argv[2] if len(sys.argv) > 2 else None
lines = open(path).read().split("\n")
start = next(i for i,l in enumerate(lines) if l.startswith("Call graph:"))
end = next(i for i,l in enumerate(lines) if l.startswith("Total number in stack"))
rx = re.compile(r"^([\s+!:|]*)(\d+) (.+?)  \(in ")
incl = collections.Counter()
stack = []  # (depth, name)
total = 0
for l in lines[start+1:end]:
    m = rx.match(l)
    if not m: continue
    depth = len(m.group(1)); n = int(m.group(2)); name = m.group(3)
    name = re.sub(r"::h[0-9a-f]{16}$", "", name)
    name = re.sub(r"\s+\+ \d+.*$", "", name)
    while stack and stack[-1][0] >= depth: stack.pop()
    if not any(s[1] == name for s in stack):
        incl[name] += n
    stack.append((depth, name))
    if depth == min(d for d,_ in stack): pass
root = max(incl.values())
for name, n in incl.most_common(int(sys.argv[3]) if len(sys.argv) > 3 else 60):
    if pat and not re.search(pat, name): continue
    print(f"{100*n/root:5.1f}% {n:6d} {name[:150]}")
