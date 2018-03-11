#!/usr/bin/env python
import sys
import math
import pygame
import random
from math import sin, cos, atan2, hypot


def dist(n1, n2):
    p1 = n1
    p2 = n2
    if n1.__class__.__name__ == 'Node':
        p1 = list(n1.point)
    if n2.__class__.__name__ == 'Node':
        p2 = list(n2.point)
    if p1 is None or p2 is None:
        return 1e200
    return hypot((p1[0]-p2[0]), (p1[1]-p2[1]))


class Node(object):
    """ Node in tree """

    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


class MyPolygon(object):
    def __init__(self, vertex_list):
        self.polygon = vertex_list[:]
        self.vertex_list = vertex_list[:]
        self.polygon.append(self.polygon[0])

    def is_in_obstacle(self, p):
        isVertex = inLine = True
        polygon = self.polygon[:]
        count = 0
        for i in range(0, len(polygon)-1):
            p1 = polygon[i]
            p2 = polygon[i+1]
            if p[0] == p1[0] and p[0] == p1[0]:
                return isVertex
            elif abs(dist(p1, p2) - dist(p1, p) - dist(p, p2)) == 0:
                return inLine
            elif (p[1] <= p1[1] and p[1] > p2[1]) or (p[1] > p1[1] and p[1] <= p2[1]):
                if p1[0] + (p[1] - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) == p[0]:
                    return True
                elif p1[0] + (p[1] - p1[1]) * (p2[0] - p1[0]) / (p2[1] - p1[1]) > p[0]:
                    count += 1
            else:
                pass
        if count % 2 == 0:
            return False
        else:
            return True
