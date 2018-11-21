#!/usr/bin/env python
import math
from itertools import permutations

#this file is just for testing python stuff

def convert_index(tup):
  return ( int(math.ceil((tup[0]+6) / 0.012)), int(math.ceil((tup[1]+4.8) / 0.012))  )

print len(range(5))

