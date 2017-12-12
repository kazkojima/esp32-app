#!/usr/bin/env python
'''
Extract NV parameter list
'''
import sys
for l in sys.stdin:
    sys.stdout.write(l[l.find('//@ ')+4:])
