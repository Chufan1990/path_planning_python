#/usr/bin/env python
import os
import rospy
import numpy as np
import yaml
import time
import math
from util import dist, Node, MyPolygon
# import constant
from rrt_v2 import RapidExpandRandomTree
from hmi_publisher import RRTVisualizer


class RRTInterface(object):
    def __ini__(self):
        super(RRTInterface, self).__init__()
        data = yaml.load(open(os.path.join(os.path.abspath(
            "/path_planning/rrt/config/", 'test.yaml'))))

        # set problem
        self.node_start = Node(None, None)
        self.node_goal = Node(None, None)
        self.pose_start_set = False
        self.pose_goal_set = False

        # set environment
        self.x_dim = data['map']['three']['x']
        self.y_dim = data['map']['three']['y']
        self.boundary = []

        # get const parameters
        self.EPSILON = rospy.get_param('/path_planning/rrt/epsilion', 1)
        self.RADIUS = rospy.get_param('/path_planning/rrt/radius', 1)
        self.NUMMODES = rospy.get_param('/path_planning/rrt/node_num', 5000)
        self.LEVEL = rospy.get_param('/path_planning/rrt/level', 1)

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
        self.node_new_sub = rospy.Subscriber(
            '/path_planning/rrt/node_new', Node, self.node_new_visualizer)
        self.tree_sub = rospy.Subscriber(
            '/path_planning/rrt/tree', Node, self.node_new_visualizer)
        # set hmi_publisher
        self.him_publisher = RRTVisualizer()

        # initialization
        self.init_boundary(data)
        self.init_obstacles(self.LEVEL)
        self.init_point_goal(data)
        self.init_point_start(data)

    def init_obstacles(self, config_num):
        obstacles = []
        print("config {}".format(config_num))
        if config_num >= 1:
            obstacles.append(self.obstacle_one)
        if config_num >= 2:
            obstacles.append(self.obstacle_two)
        if config_num >= 3:
            obstacles.append(self.obstacle_three)
        if config_num >= 4:
            obstacles.append(self.obstacle_four)
        return np.array(obstacles)

    def init_point_goal(self, data):
        point_goal = data['points']['goal']
        if point_goal[0] < 0 or point_goal[0] > self.x_dim or point_goal[1] < 0 or point_goal[1] > self.y_dim:
            rospy.logwarn("Goal point out of boundary!")
            return False
        else:
            self.node_init = Node(point_goal, None)
            return True

    def init_point_start(self, data):
        point_start = data['points']['start']
        if point_start[0] < 0 or point_start[0] > self.x_dim or point_start[1] < 0 or point_start[1] > self.y_dim:
            rospy.logwarn("Start point out of boundary!")
            return False
        else:
            self.node_goal = Node(point_start, None)
            return True

    def init_boundary(self, data):
        self.boundary.append(data['map']['one'])
        self.boundary.append(data['map']['two'])
        self.boundary.append(data['map']['three'])
        self.boundary.append(data['map']['four'])

    def update(self):
        rrt_solver = RapidExpandRandomTree(
            self.node_start, self.node_goal, self.obstacles, self.x_dim, self.y_dim, self.RADIUS, self.EPSILON, self.NUMMODES)
        rrt_solver.update()
        # TODO: pass node_new/tree/boundary/obstacles to hmi

    def node_new_visualizer(self, data):
        self.him_publisher.plot_node_new(data.data.point, 0)
        self.him_publisher.plot_boundary(self.boundary, 0)
        self.him_publisher.plot_obstacles(self.obstacles, 0)

    def tree_visualizer(self, data):
        self.him_publisher.plot_tree(data.data.point, 0)
        self.him_publisher.plot_boundary(self.boundary, 0)
        self.him_publisher.plot_obstacles(self.obstacles, 0)
