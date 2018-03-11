#!/usr/bin/env python
import os
import rospy
import math
import numpy as np
from util import dist, atan2, sin, cos, random, Node
from geometry_msgs.msg import Polygon, Point32


class RapidExpandRandomTree:
    def __init__(self, node_start, node_goal, pose_start_set, pose_goal_set, obstacles, x_dim, y_dim, RADIUS, EPSILON, NUMMODES):
        self.state = 'init'
        self.pose_start_set = pose_start_set
        self.pose_goal_set = pose_goal_set
        self.node_start = node_start
        self.node_goal = node_goal

        self.x_dim = x_dim
        self.y_dim = y_dim

        self.obstacles = obstacles
        self.nodes = []

        self.RADIUS = RADIUS
        self.EPSILON = EPSILON
        self.NUMMODES = NUMMODES

        self.pub_tree = rospy.Publisher(
            '/path_planning/rrt/tree', Polygon, queue_size=100)
        self.pub_path = rospy.Publisher(
            '/path_planning/rrt/path', Polygon, queue_size=100)
        self.rate = rospy.Rate(10)
        # self.rate_path = rospy.Rate(1)

    def update(self):
        while not rospy.is_shutdown():
            if self.state == 'init':
                rospy.loginfo("state is {}".format(self.state))
                if self.pose_goal_set == False or self.pose_start_set == False or self.node_start == None or self.node_goal == None:
                    rospy.logwarn("Start or goal point not set yet")
                else:
                    self.nodes = [self.node_start]
                    self.state = 'build tree'
            elif self.state == 'build tree':
                rospy.loginfo("state is {}".format(self.state))
                self.state, self.nodes = self.build_tree()
            elif self.state == 'complete':
                rospy.loginfo("state is {}".format(self.state))
                node_current = self.nodes[-1]
                path = []
                while node_current.parent != None:
                    # pose_current = Point32(
                    #     node_current.point[0], node_current.point[1], 0.0)
                    pose_parrent = Point32(
                        node_current.parent.point[0], node_current.parent.point[1], 0.0)
                    path.append(pose_parrent)
                    # path.append(pose_current)
                    self.pub_path.publish(Polygon(path))
                    node_current = node_current.parent
                    self.rate.sleep()
                self.state = 'done'
            elif self.state == 'incomplete':
                rospy.loginfo("state is {}".format(self.state))
                rospy.loginfo("Cannot find path!")
            else:
                break
            self.rate.sleep()
            if rospy.is_shutdown():
                del self.nodes[:]

    def build_tree(self):
        count = 0
        tree = []
        nodes = self.nodes[:]
        node_goal = self.node_goal
        state = self.state
        while count < self.NUMMODES and state != 'complete':
            node_new = self.get_next_node(nodes)
            count += 1
            point_new = Point32(node_new.point[0], node_new.point[1], 0.0)
            point_parent = Point32(
                node_new.parent.point[0], node_new.parent.point[1], 0.0)
            nodes.append(node_new)
            tree.append(point_parent)
            tree.append(point_new)
            self.pub_tree.publish(Polygon(tree))
            state = self.reach_goal(node_new, node_goal, state)
            self.rate.sleep()
        return state, nodes

    def reach_goal(self, node_new, node_goal, state):
        if dist(node_new, node_goal) <= self.RADIUS:
            state = 'complete'
        return state

    def get_random(self):
        return [random.random()*self.x_dim, random.random()*self.y_dim]

    def get_random_clear(self):
        p = self.get_random()

        while self.collides([p]) == True:
            p = self.get_random()
        return p

    def collides(self, points):
        for p in points:
            for obs in self.obstacles:
                if obs.is_in_obstacle(p) == True:
                    return True
        return False

    def step_from_to(self, n1, n2):
        if n1.__class__.__name__ == "Node":
            p1 = n1.point
        else:
            p1 = n1
        if n2.__class__.__name__ == "Node":
            p2 = n2.point
        else:
            p2 = n2

        if dist(n1, n2) > self.EPSILON:
            theta = atan2(p2[1] - p1[1], p2[0] - p1[0])
            p_new = [p1[0] + self.EPSILON *
                     cos(theta), p1[1] + self.EPSILON*sin(theta)]
        else:
            p_new = p2

        diff = int(max(abs(p1[0]-p_new[0]), abs(p1[1]-p_new[1]), 2))

        point_array = zip(np.linspace(
            p1[0], p_new[0], diff), np.linspace(p1[1], p_new[1], diff))

        return p_new, point_array

    def get_next_node(self, nodes):
        found_next = False
        # point_test = None
        point_for_sure = None
        node_temp_nearest = None
        while found_next == False:
            point_rand = self.get_random_clear()
            node_temp_nearest = nodes[0]
            for node in nodes:
                if dist(node.point, point_rand) <= dist(node_temp_nearest.point, point_rand):
                    point_candidate, point_array = self.step_from_to(
                        node.point, point_rand)
                    if self.collides(point_array) == False:
                        # point_test = point_rand
                        node_temp_nearest = node
                        point_for_sure = point_candidate
                        found_next = True
        node_new = Node(point_for_sure, node_temp_nearest)
        return node_new

    def test(self):
        point1 = [980.0345696465944, 723.5366155960791]
        point2 = [481.2932366856792, 757.4846282210185]
        point3 = [9.08469719254, 37.7420706336]
        point4 = [17.2527181564, 43.4821910282]

        print(dist(point1, point3), dist(point1, point4))
        point_candidate1, point_array1 = self.step_from_to(point3, point1)
        point_candidate2, point_array2 = self.step_from_to(point4, point1)
        print(self.collides(point_array1), self.collides(point_array2))
        print(point_candidate1, point_candidate2)
