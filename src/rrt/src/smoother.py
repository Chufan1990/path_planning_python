import rospy
import numpy as np
from util import dist, Node


class RRTSmoother(object):
    def __init__(self, path, obstacles):
        super(RRTSmoother, self).__init__()
        self.path = path[:]
        self.obstacles = obstacles[:]

    def update(self):
        node_choose = self.path[0]
        path_optimized = [self.path[0]]
        while node_choose.parent != None:
            node_nearest = self.path_smoother(node_choose)
            if node_nearest != None:
                node_next = Node(node_nearest.point, node_choose)
                node_choose = node_nearest
                path_optimized.append(node_next)
            else:
                node_choose = node_choose.parent
                node_next = node_choose
            path_optimized.append(node_next)
        return path_optimized

    def path_smoother(self, node_choose):
        node_nearest = None
        for node in self.path[:]:
            if self.path_collides(node, node_choose) == False:
                if dist(node, node_choose) < self.path_distance_calculator(node, node_choose):
                    node_nearest = node
                    # node_choose = node
        return node_nearest

    def path_distance_calculator(self, node, node_choose):
        if dist(node.parent, node_choose) != 0:
            return dist(node, self.path_distance_calculator(node.parent, node_choose))
        else:
            return 0

    def path_collides(self, node, node_choose):
        checker_num = int(max(abs(
            node.point[0]-node_choose.point[0]), abs(node.point[1] - node_choose.point[1]), 3.0))
        checkers = zip(np.linspace(node.point[0], node_choose.point[0], checker_num), np.linspace(
            node.point[1], node_choose.point[1], checker_num))
        if self.collides(checkers) == True:
            return True
        return False

    def collides(self, points):
        for p in points[:]:
            for obs in self.obstacles[:]:
                if obs.is_in_obstacle(p) == True:
                    return True
        return False
