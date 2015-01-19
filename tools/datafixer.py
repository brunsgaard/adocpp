#!/usr/bin/env python

import sys

fname = sys.argv[1]

with open(fname) as f:
    res = sorted(list(set([tuple(sorted(map(int,filter(None, l.split())))) for l in f.readlines()])))

with open(fname, 'w') as f:
    for v1, v2 in res:
        f.write('{} {}\n'.format(v1, v2))



