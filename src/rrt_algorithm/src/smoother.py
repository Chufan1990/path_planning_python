import rospy
import numpy as np
from util import dist, Node
from geometry_msgs.msg import Point32


class RRTSmoother(object):
    def __init__(self, nodes, obstacles, RATE):
        super(RRTSmoother, self).__init__()
        self.nodes = nodes
        self.obstacles = obstacles
        self.rate = rospy.Rate(int(RATE))

    def update(self):
        node_choose = self.nodes[0]
        path_optimized = [Point32(self.nodes[0].point[0], self.nodes[0].point[1], 0.0)]
        path_optimized  = []
        node_optimized = [self.nodes[0]]
        while node_choose.parent != None:
            node_nearest = self.path_smoother(node_choose)
            if node_nearest != None:
                node_next = Node(node_choose.point, node_nearest)
                node_choose = node_nearest
            else:
                node_choose = node_choose.parent
                node_next = node_choose
            point_next = Point32(node_next.point[0], node_next.point[1], 0.0)
            path_optimized.append(point_next)
            node_optimized.append(node_next)
            self.rate.sleep()
        path_optimized.append(Point32(self.nodes[-1].point[0], self.nodes[-1].point[1], 0.0))
        return path_optimized, node_optimized

    def path_smoother(self, node_choose):
        node_nearest = None
        nodes = self.nodes[:]
        for node in reversed(nodes):
            if self.path_collides(node, node_choose) == False:
                if dist(node, node_choose) <= self.path_distance_calculator(node, node_choose):
                    node_nearest = node
                    break
        return node_nearest

    def path_distance_calculator(self, node, node_choose):
        if node_choose.parent != None and dist(node, node_choose) != 0:
            return dist(node_choose, node_choose.parent) + self.path_distance_calculator(node, node_choose.parent)
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

    def test(self):
        for node in self.nodes:
            rospy.loginfo("distance1 {}".format(self.path_distance_calculator(node, self.nodes[0])))
            rospy.loginfo("distance2 {}".format(dist(node, self.nodes[0])))


