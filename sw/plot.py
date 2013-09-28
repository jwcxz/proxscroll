#!/usr/bin/env python2

import sys
import matplotlib.pyplot as plt

f = open(sys.argv[1], 'r');
d = [];
for line in f:
    d.append(int(line.strip()));

plt.plot(d);
plt.show();
