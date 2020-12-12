#! /usr/bin/env python

class MyPoint:
    def __init__(self, row, col, f=0, g=0, h=0):
        self.row = row
        self.col = col
        self.f = f
        self.g = g
        self.h = h

    def setF(self):
        self.f = self.g + self.h

class TreeNode:
    def __init__(self, pose, child=[], parent=None):
        self.pose = pose
        self.child = child
        self.parent = parent