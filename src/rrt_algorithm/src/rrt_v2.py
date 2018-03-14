#!/usr/bin/env python
import os
import rospy
import math
import numpy as np
from util import dist, atan2, sin, cos, random, Node
from geometry_msgs.msg import Polygon, Point32
from smoother import RRTSmoother


class RapidExpandRandomTree:
    def __init__(self, node_start, node_goal, pose_start_set, pose_goal_set, obstacles, x_dim, y_dim, RADIUS, EPSILON, NUMMODES, RATE):
        self.state = 'initializing'
        self.pose_start_set = pose_start_set
        self.pose_goal_set = pose_goal_set
        self.node_start = node_start
        self.node_goal = node_goal

        self.x_dim = x_dim
        self.y_dim = y_dim

        self.obstacles = obstacles
        self.nodes = []
        self.node_last = Node(None, None)

        self.RADIUS = RADIUS
        self.EPSILON = EPSILON
        self.NUMMODES = NUMMODES
        self.RATE = RATE
        
        self.pub_tree = rospy.Publisher(
            '/path_planning/rrt/tree', Polygon, queue_size=100)
        self.pub_path = rospy.Publisher(
            '/path_planning/rrt/path', Polygon, queue_size=100)
        self.pub_path_optimized = rospy.Publisher('/path_planning/rrt/path_optimized', Polygon, queue_size =1)
        self.rate = rospy.Rate(int(RATE))

    def update(self):
        while not rospy.is_shutdown():
            if self.state == 'initializing':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                if self.pose_goal_set == False or self.pose_start_set == False or self.node_start == None or self.node_goal == None:
                    rospy.logwarn("Start or goal point not set yet")
                else:
                    self.nodes = [self.node_start]
                    self.state = 'building tree'
            elif self.state == 'building tree':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                self.state, self.node_last = self.build_tree()
            elif self.state == 'completed':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                node_current = self.node_last
                path = []
                self.nodes = [node_current]
                while node_current.parent != None:
                    pose_parrent = Point32(
                        node_current.parent.point[0], node_current.parent.point[1], 0.0)
                    path.append(pose_parrent)
                    self.pub_path.publish(Polygon(path))
                    node_current = node_current.parent
                    self.nodes.append(node_current)
                    self.rate.sleep()
                self.state = "testing"
            elif self.state == 'testing':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                for node in self.nodes:
                    rospy.loginfo("{} ".format(node.point))
                self.state = 'optimizating'
            elif self.state == 'optimizating':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                path_smoother = RRTSmoother(self.nodes, self.obstacles, self.RATE)
                # path_smoother.test()
                path_optimized, __ = path_smoother.update()
                self.pub_path_optimized.publish(Polygon(path_optimized))
                self.state = 'Exiting'
            elif self.state == 'incompleted':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                rospy.logwarn("Cannot find path!")
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
        while count < self.NUMMODES and not state == 'completed':
            node_new = self.get_next_node(nodes)
            count += 1
            point_new = Point32(node_new.point[0], node_new.point[1], 0.0)
            point_parent = Point32(
                node_new.parent.point[0], node_new.parent.point[1], 0.0)
            nodes.append(node_new)
            tree.append(point_parent)
            tree.append(point_new)
            self.pub_tree.publish(Polygon(tree))
            state, node_last = self.reach_goal(node_new, node_goal, state)
            self.rate.sleep()
        return state, node_last

    def reach_goal(self, node_new, node_goal, state):
        if dist(node_new, node_goal) <= self.RADIUS:
            state = 'completed'
        return state, node_new

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

class RapidExpandRandomTreeStar:
    def __init__(self, node_start, node_goal, pose_start_set, pose_goal_set, obstacles, x_dim, y_dim, RADIUS, EPSILON, NUMMODES, RATE):
        self.state = 'initializing'
        self.pose_start_set = pose_start_set
        self.pose_goal_set = pose_goal_set
        self.node_start = node_start
        self.node_goal = node_goal

        self.x_dim = x_dim
        self.y_dim = y_dim

        self.obstacles = obstacles
        self.nodes = []
        self.node_last = Node(None, None)

        self.RADIUS = RADIUS
        self.EPSILON = EPSILON
        self.NUMMODES = NUMMODES
        self.RATE = RATE
        
        self.pub_tree = rospy.Publisher(
            '/path_planning/rrt/tree', Polygon, queue_size=100)
        self.pub_path = rospy.Publisher(
            '/path_planning/rrt/path', Polygon, queue_size=100)
        self.pub_path_optimized = rospy.Publisher('/path_planning/rrt/path_optimized', Polygon, queue_size =1)
        self.rate = rospy.Rate(int(RATE))

    def update(self):
        while not rospy.is_shutdown():
            if self.state == 'initializing':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                if self.pose_goal_set == False or self.pose_start_set == False or self.node_start == None or self.node_goal == None:
                    rospy.logwarn("Start or goal point not set yet")
                else:
                    self.nodes = [self.node_start]
                    self.state = 'building tree'
            elif self.state == 'building tree':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                self.state, self.node_last = self.build_tree()
            elif self.state == 'completed':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                node_current = self.node_last
                path = []
                self.nodes = [node_current]
                while node_current.parent != None:
                    pose_parrent = Point32(
                        node_current.parent.point[0], node_current.parent.point[1], 0.0)
                    path.append(pose_parrent)
                    self.pub_path.publish(Polygon(path))
                    node_current = node_current.parent
                    self.nodes.append(node_current)
                    self.rate.sleep()
                self.state = "testing"
            elif self.state == 'testing':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                for node in self.nodes:
                    rospy.loginfo("{} ".format(node.point))
                self.state = 'optimizating'
            elif self.state == 'optimizating':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                path_smoother = RRTSmoother(self.nodes, self.obstacles, self.RATE)
                # path_smoother.test()
                path_optimized, __ = path_smoother.update()
                self.pub_path_optimized.publish(Polygon(path_optimized))
                self.state = 'Exiting'
            elif self.state == 'incompleted':
                rospy.loginfo("="*len("Current State:   {}".format(self.state)))
                rospy.loginfo("Current State:   {}".format(self.state))
                rospy.logwarn("Cannot find path!")
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
        while count < self.NUMMODES and not state == 'completed':
            node_new = self.get_next_node(nodes)
            count += 1
            point_new = Point32(node_new.point[0], node_new.point[1], 0.0)
            point_parent = Point32(
                node_new.parent.point[0], node_new.parent.point[1], 0.0)
            nodes.append(node_new)
            tree.append(point_parent)
            tree.append(point_new)
            self.pub_tree.publish(Polygon(tree))
            state, node_last = self.reach_goal(node_new, node_goal, state)
            self.rate.sleep()
        return state, node_last

    def reach_goal(self, node_new, node_goal, state):
        if dist(node_new, node_goal) <= self.RADIUS:
            state = 'completed'
        return state, node_new

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

    # def RRTStarSearch(self):
    #     tree = set()
    #     nodes = set(self.nodes[:])
    #     for i in range(self.NUMMODES):
    #         node_new = self.get_next_node(nodes)
    #         nearest_set = self.find_nearest_set(node_new)
    #         node_min = self.find_min_node(node_new)
    #         node_new.parent = node_min
    #         nodes.append(node_new)
    #         tree.append((node_min.point,node_new.point))
    #         nodes, tree = self.rewire(nearest_set, node_min. node_new, nodes, tree)
    #         self.pub_tree.publish(Polygon(tree))
    #         __, node_last = self.reach_goal(node_new, self.node_goal, state="")
            

    # def find_nearest_set(self, node_new):
    #     node_set=set()
    #     ball_ra_dius = self.find_ball_radius()
    #     for node in self.nodes:
    #         if dist(node, node_new) < ball_radius:
    #             points.add(node)
    #     return node_set

    # def find_ball_radius(self):
    #     unit_ball_volume = math.pi
    #     n = len(self.nodes)
    #     dimensions = 2.0 
    #     gamma = ((1 + 1/dimensions) * ((math.hypot(self.x_dim, self.y_dim)**2)/unit_ball_volume))
    #     ball_radius = min(2*(gamma *log(n)/n)**(1/dimensions), self.EPSILON)
    #     return ball_radius

    # def find_min_node(self, nearest_set, node_new):
    #     node_min = node_nearest
    #     cost_min = self.cost(node_nearest) + self.lincost(node_nearest, node_new)
    #     for node in self.nodes:
    #         if self.isEdgeCollisionFree(node, node_new):
    #             cost_temp = self.cost(node) + self.lincost(node, node_new)
    #             if cost_temp < cost_min:
    #                 cost_min = cost_temp
    #                 node_min = node
    #     return node_min

    # def isEdgeCollisionFree(self, n1, n2):
    #     checker_num = max(abs(n1.point[0] - n2.point[0]), abs(n1.pointp[1] - n2.point[1]),3)
    #     checkers = zip(np.linspace(n1.point[0], n2.point[0], checker_num), np.linspace(n1.point[1], n2.point[1], checker_num))
    #     if self.collides(checkers) == True:
    #         return False
    #     return True

    # def cost(self, node):
    #     path, cost = self.find_path(self.node_start, node)

    # def lincost(self, n1, n2):
    #     return dist(n1, n2)

    # def find_path(self, node_start, node_now):
    #     count = 0
    #     node_current = node_now
    #     node_target = node_start
    #     path = []
    #     while node_current != node_target:
    #         path.append(node_current)
    #         node_current = node_current.parent
    #     path.reverse()
    #     return path, len(path)

    # def rewire(self, nearest_set, node_min, node_new, nodes, tree):
    #     for node in nodes - [node_min]:
    #         if self.isEdgeCollisionFree(node, node_new):
    #             if self.cost(node) > sefl.cost(node_new) + self.lincost(node, node_new):
    #                 tree.discard((node_new.point, node_new.parent.point))
    #                 tree.discard((node_new.parent,point, node_new.point))
    #                 nodes.discard(node_new)
    #                 node_new.parent = node
    #                 tree.add((node.point, node_new.point))
    #                 node.add(node_new)
    #     return nodes, tree