#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np

def xfrange(start, stop, step):
    i = 0
    while start + i * step < stop:
        yield start + i * step
        i += 1

x = []
y = []

sigma = 0.1
r = 10 

for i in xfrange(0, 12, 0.1):
  p = 1/(math.sqrt(2*math.pi*sigma))*math.exp(-(x-r)^2/(2*sigma))
  x.append (i)
  y.append (p)

plt.plot(x,y)
plt.ylabel('some numbers')
plt.show()