#!/usr/bin/python3

import sys

if len(sys.argv) > 1:
    print("Usage: cat file1 file2 | %s " % sys.argv[0])
    print("       Combines multiple results into one.")
    print("       The lines should be ordered in ascending priority,")
    print("       i.e. last line has the highest priority.")
    sys.exit()

f = sys.stdin
g = sys.stdout

res = dict()
for line in f:
    words = line.split()
    if len(words) > 0:
        res[words[0]] = line

for line in sorted(res.values()):
    g.write(line)
