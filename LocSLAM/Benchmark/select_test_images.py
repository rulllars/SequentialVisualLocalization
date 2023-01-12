#!/usr/bin/python3

import sys

if len(sys.argv) < 2:
    print("Usage: %s test_image_file [in_file [out_file]]" % sys.argv[0])
    sys.exit()
try:
    if len(sys.argv) >= 3:
        f = open(sys.argv[2], 'r')
    else:
        f = sys.stdin

    if len(sys.argv) >= 4:
        g = open(sys.argv[3], 'w')
    else:
        g = sys.stdout

    testimages = set(line.strip() for line in open(sys.argv[1], 'r'))
    queryimages = set()

    for line in f:
        words = line.split()
        queryimages.add(words[0])
        if words[0] in testimages:
            g.write(" ".join(words) + "\n")

    missing = testimages - queryimages
    if len(missing) > 0:
        print("Warning: %d images are not present in the query file list." % len(missing))


finally:
    if f is not sys.stdin:
        f.close()

    if g is not sys.stdout:
        g.close()
