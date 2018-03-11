#!/usr/bin/env python
import os
import rospy
import numpy as np
import yaml
import time
import math
from util import dist, Node, MyPolygon
from geometry_msgs.msg import Point, Polygon
from rrt_v2 import RapidExpandRandomTree
from hmi_publisher import RRTVisualizer
import rospkg


class RRTInterface(object):
    def __init__(self):
        super(RRTInterface, self).__init__()
        rospack = rospkg.RosPack()
        data = yaml.load(
            open(os.path.join(rospack.get_path('rrt'), 'config/test.yaml')))

        # set problem
        self.node_start = Node(None, None)
        self.node_goal = Node(None, None)
        self.pose_start_set = False
        self.pose_goal_set = False

        # set environment
        self.x_dim = data['map']['three'][0]
        self.y_dim = data['map']['three'][1]
        self.boundary = []

        # get const parameters
        self.EPSILON = rospy.get_param('~epsilion', 10)
        self.RADIUS = rospy.get_param('~radius', 1)
        self.NUMMODES = rospy.get_param('~node_num', 5000)
        self.LEVEL = rospy.get_param('~level', 1)

        # set solver
        self.state = 'init'
        self.nodes = []

        # set obstacles
        self.obstacles = []
        self.obstacle_one = data["obstacles"]['first']
        self.obstacle_two = data["obstacles"]['second']
        self.obstacle_three = data["obstacles"]['third']
        self.obstacle_four = data["obstacles"]['forth']

        # set subscriber
        self.sub_tree = rospy.Subscriber(
            '/path_planning/rrt/tree', Polygon, self.tree_visualizer)
        self.sub_path = rospy.Subscriber(
            '/path_planning/rrt/path', Polygon, self.path_visualizer)
        # set hmi_publisher
        self.hmi_publisher = RRTVisualizer()

        # initialization
        self.init_boundary(data)
        self.init_obstacles(self.LEVEL)
        self.pose_goal_set = self.init_point_goal(data)
        self.pose_start_set = self.init_point_start(data)

    def init_obstacles(self, config_num):
        obstacles = []
        # print("config {}".format(config_num))
        if config_num >= 1:
            obstacles.append(MyPolygon(self.obstacle_one))
            rospy.loginfo("obstacle type: {}".format(type(self.obstacle_one)))
        if config_num >= 2:
            obstacles.append(MyPolygon(self.obstacle_two))
            rospy.loginfo("obstacle type: {}".format(type(self.obstacle_two)))
        if config_num >= 3:
            obstacles.append(MyPolygon(self.obstacle_three))
            rospy.loginfo("obstacle type: {}".format(
                type(self.obstacle_three)))
        if config_num >= 4:
            obstacles.append(MyPolygon(self.obstacle_four))
            rospy.loginfo("obstacle type: {}".format(type(self.obstacle_four)))
        self.obstacles = obstacles

    def init_point_goal(self, data):
        point_goal = data['points']['goal']
        if point_goal[0] < 0 or point_goal[0] > self.x_dim or point_goal[1] < 0 or point_goal[1] > self.y_dim:
            rospy.logwarn("Goal point out of boundary!")
            return False
        else:
            self.node_goal = Node(point_goal, None)
            return True

    def init_point_start(self, data):
        point_start = data['points']['start']
        if point_start[0] < 0 or point_start[0] > self.x_dim or point_start[1] < 0 or point_start[1] > self.y_dim:
            rospy.logwarn("Start point out of boundary!")
            return False
        else:
            self.node_start = Node(point_start, None)
            return True

    def init_boundary(self, data):
        self.boundary.append(data['map']['one'])
        self.boundary.append(data['map']['two'])
        self.boundary.append(data['map']['three'])
        self.boundary.append(data['map']['four'])

    def update(self):
        rrt_solver = RapidExpandRandomTree(
            self.node_start, self.node_goal, self.pose_start_set, self.pose_goal_set, self.obstacles, self.x_dim, self.y_dim, self.RADIUS, self.EPSILON, self.NUMMODES)
        rrt_solver.update()
        # TODO: pass node_new/tree/boundary/obstacles to hmi

    def tree_visualizer(self, data):
        # rospy.loginfo("tree_visualizer working correct")
        self.hmi_publisher.plot_tree(data)
        self.hmi_publisher.plot_boundary(
            self.boundary_for_visualization(self.boundary), 0.0)
        self.hmi_publisher.plot_obstacles(
            self.obstacles_for_visualization(self.obstacles), 0.0)
        self.hmi_publisher.plot_points(self.points_visualizer(), 0.0)

    def path_visualizer(self, data):
        # rospy.loginfo("path_visualizer working correct")
        self.hmi_publisher.plot_path(data)
        self.hmi_publisher.plot_boundary(
            self.boundary_for_visualization(self.boundary), 0.0)
        self.hmi_publisher.plot_obstacles(
            self.obstacles_for_visualization(self.obstacles), 0.0)
        self.hmi_publisher.plot_points(self.points_visualizer(), 0.0)

    def points_visualizer(self):
        return [self.node_start.point, self.node_goal.point]

    def boundary_for_visualization(self, boundary):
        b = boundary[:]
        b.append(b[0])
        return b

    def obstacles_for_visualization(self, obstacles):
        o = obstacles
        hmi_x = []
        hmi_y = []
        for obs in o:
            for i in range(0, len(obs.polygon)-1):
                hmi_x += [obs.polygon[i][0], obs.polygon[i+1][0]]
                hmi_y += [obs.polygon[i][1], obs.polygon[i+1][1]]
        return zip(hmi_x, hmi_y)

    def test(self):
        rrt_solver = RapidExpandRandomTree(self.node_start, self.node_goal, self.pose_start_set, self.pose_goal_set,
                                           self.obstacles, self.x_dim, self.y_dim, self.RADIUS, self.EPSILON, self.NUMMODES)
        rrt_solver.test()
