import os
import rospy
import math
import numpy as np
from util import *


class RapidExpandRandomTree:
    def __init__(self, node_start, node_goal, obstacles, x_dim, y_dim, RADIUS, EPSILON, NUMMODES):
        self.state = 'init'
        self.pose_start_set = False
        self.pose_goal_set = False
        self.node_start = node_start
        self.node_goal = node_goal

        self.x_dim = x_dim
        self.y_dim = y_dim

        self.obstacles = obstacles
        self.nodes = []

        self.RADIUS = RADIUS
        self.EPSILON = EPSILON
        self.NUMMODES = NUMMODES

        self.pub_node_new = rospy.Publisher(
            '/path_planning/rrt/node_new', Node, queue_size=1)
        self.pub_path = rospy.Publisher(
            '/path_planning/rrt/node_current', Node, queue_size=1)

    def update(self):
        while not rospy.is_shutdown():
            if self.state == 'init':
                if self.pose_goal_set == False or self.pose_start_set == False or self.node_start == None or self.node_goal == None:
                    rospy.logwarn("Start or goal point not set yet")
                else:
                    self.state = 'build tree'
            elif self.state == 'build tree':
                # count = 0
                # while count < self.NUMMODES and self.state != 'complete':
                #     node_new = self.get_next_node(self.nodes)
                #     count += 1
                #     yield node_new
                #     self.nodes.append(node_new)
                #     self.state = self.reach_goal(self.nodes, self.state)
                #     # TODO: yield
                self.state, self.nodes = self.build_tree(self.nodes, self.node_goal, self.state)
            elif self.state == 'complete':
                node_current = self.nodes[-1]
                while node_current.parent != None:
                    # yield node_current
                    self.pub_path.publish(node_current)
                    node_current = node_current.parent
                self.state = 'init'
            elif self.state == 'incomplete':
                rospy.loginfo("Cannot find path!")
            else:
                pass

    def build_tree(self, nodes, node_goal, state):
        count = 0
        while count < self.NUMMODES and state != 'complete':
            node_new = self.get_next_node(nodes)
            count += 1
            # yield node_new
            self.pub_node_new.publish(node_new)
            nodes.append(node_new)
            state = self.reach_goal(node_new, node_goal, state)
        return state, nodes


    def reach_goal(self, node_new, node_goal, state):
        if dist(node_new, node_goal) <= self.RADIUS:
            state = 'complete'
        return state

    def get_random(self):
        return [random.random()*self.x_dim, random.random()*self.y_dim]

    def get_random_clear(self):
        p = self.get_random()
        while self.collides(p) == True:
            p = self.get_random()
        return p

    def collides(self, points):
        for p in points:
            for obs in self.obstacles:
                if obs.is_in_obstacle(p) == True:
                    return True
        return False

    def step_from_to(self, n1, n2):
        p1 = n1.point
        p2 = n2.point
        if dist(n1, n2) > self.EPSILON:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            p_new = [p1[0] + self.EPSILON *
                     cos(theta), p1[1] + self.EPSILON*sin(theta)]
        point_array = zip(np.linspace(
            p1[0], p2[0], 100), np.linspace(p1[1], p2[1], 100))
        return p_new, point_array

    def get_next_node(self, nodes):
        found_next = False
        while found_next == False:
            point_rand = self.get_random_clear()
            node_temp_nearest = nodes[0]
            for node in nodes:
                if node is not None and node.point is not None:
                    if dist(node.point, point_rand) <= dist(node_temp_nearest.point, point_rand):
                        point_candidate, point_array = self.step_from_to(
                            node.point, point_rand)
                        if self.collides(point_array) == False:
                            node_temp_nearest = node
                            found_next = True
        node_new = Node(point_candidate, node_temp_nearest)
        return node_new
