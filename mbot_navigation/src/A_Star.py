#! /usr/bin/env python
import rospy
import copy


class A_Star:
    def __init__(self, ROWS, COLS, ocMapData, LINECOST=5, ARCOST=7, origin=13.8, resolution=0.05):
        self.ROWS = ROWS
        self.COLS = COLS
        self.LINECOST = LINECOST
        self.ARCOST = ARCOST
        self.ocMapData = ocMapData
        self.origin = origin
        self.resolution = resolution

    def generateMap(self):
	map = []
        tmp = []
        ocMapData = list(self.ocMapData)
        ocMapData = ocMapData[::-1]
        for i in range(1, len(ocMapData) + 1):
            if not int(ocMapData[i - 1]) == 0:
                ocMapData[i - 1] = 1
            tmp.append(int(ocMapData[i - 1]))
            if i % 576 == 0:
                map.append(tmp[::-1])
                tmp = []
        return map

    def getIndex(self, X, Y):
        tmpX = copy.copy(X)
        tmpY = copy.copy(Y)
        if X < 0 < Y:
            tmpX = int((self.origin + tmpX) / self.resolution)
            tmpY = int(abs(self.origin - tmpY) / self.resolution)
        if X > 0 and Y > 0:
            tmpX = int((self.origin + tmpX) / self.resolution)
            tmpY = int(abs(self.origin - tmpY) / self.resolution)
        if X > 0 > Y:
            tmpX = int((self.origin + tmpX) / self.resolution)
            tmpY = int(abs((-tmpY) + self.origin) / self.resolution)
        if X < 0 and Y < 0:
            tmpX = int((self.origin + tmpX) / self.resolution)
            tmpY = int(abs((-tmpY) + self.origin) / self.resolution)
        if X == 0 and Y == 0:
            tmpX = int((self.origin + tmpX) / self.resolution)
            tmpY = int((self.origin + tmpY) / self.resolution)
        return int(tmpX), int(tmpY)

    def getH(self, endPose, currentPose):
        row = endPose.row - currentPose.row if endPose.row > currentPose.row else currentPose.row - endPose.row
        col = endPose.col - currentPose.col if endPose.col > currentPose.col else currentPose.col - endPose.col

        return (row + col) * self.LINECOST

    def needAdd(self, currentPose, map, pathMap):
        if currentPose.row >= self.ROWS or currentPose.row < 0 or currentPose.col >= self.COLS or currentPose.col < 0:
            return False
        if 1 == map[currentPose.row][currentPose.col]:
            return False
        for i in range(1, 10):
            if 1 == map[currentPose.row + i][currentPose.col]:
                return False
            if 1 == map[currentPose.row - i][currentPose.col]:
                return False
            if 1 == map[currentPose.row][currentPose.col + i]:
                return False
            if 1 == map[currentPose.row][currentPose.col - i]:
                return False

            if 1 == map[currentPose.row + i][currentPose.col - i]:
                return False
            if 1 == map[currentPose.row + i][currentPose.col + i]:
                return False
            if 1 == map[currentPose.row - i][currentPose.col + i]:
                return False
            if 1 == map[currentPose.row - i][currentPose.col - i]:
                return False
        if 1 == pathMap[currentPose.row][currentPose.col]:
            return False
        return True

    def getCoordinate(self, row, col):
        tmpRow = -((row * self.resolution) - self.origin)
        tmpCol = (col * self.resolution) - self.origin
        return tmpRow, tmpCol
