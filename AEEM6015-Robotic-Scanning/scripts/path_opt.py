#!/usr/bin/env python
import numpy as np
from math import *#sin, cos, acos, asin, radians
from sys import maxsize
import tsp

class TSPOpt:
    # graph, cost/distance from node i to node j 2d map
    # size number of nodes
    def __init__(self,graph,size):
        self.graph = graph
        self.size = size

    def optimize(self):
        r = range(self.size)
        dist = {(i,j):self.graph[i][j] for i in r for j in r}
        t = tsp.tsp(r,dist)
        dist = t[0] # shortest distance
        path = t[1] # path
        return dist, path
