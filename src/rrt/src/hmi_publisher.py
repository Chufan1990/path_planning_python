#!/usr/bin/env python
import rospy
import numpy as np
import math
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class RRTVisualizer:
    def __init__(self):
        # self.sub_way_point = rospy.Subscriber()

        self.pub_tree = rospy.Publisher(
            '/path_planning/viz/tree', Marker, queue_size=1)
        self.pub_path = rospy.Publisher(
            '/path_planning/viz/path', Marker, queue_size=1)
        self.pub_obstacles = rospy.Publisher(
            '/path_planning/viz/obstacles', Marker, queue_size=1)
        self.pub_boudary = rospy.Publisher(
            '/path_planning/viz/boundary', Marker, queue_size=1)
        self.pub_points = rospy.Publisher(
            '/path_planning/viz/points', Marker, queue_size=1)
        self.pub_path_optimized = rospy.Publisher(
            '/path_planning/viz/path_optimized', Marker, queue_size=1)

    def plot_tree(self, polygon):
        # rospy.loginfo("polygon: {}".format(polygon))
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(20)
        marker.type = Marker.LINE_LIST
        marker.id = 0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 110/255.0
        marker.color.g = 110/255.0
        marker.color.b = 110/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for point in polygon.points:
            p = Point()
            p.x = point.x
            p.y = point.y
            p.z = point.z
            marker.points.append(p)
        self.pub_tree.publish(marker)

    def plot_path(self, polygon):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(20)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 255/255.0
        marker.color.g = 10/255.0
        marker.color.b = 10/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for point in polygon.points:
            p = Point()
            p.x = point.x
            p.y = point.y
            p.z = point.z
            marker.points.append(point)
        marker.points.append(p)
        self.pub_path.publish(marker)

    def plot_obstacles(self, obstacles, z):
        # rospy.loginfo("obstacles: {}".format(type(obstacles)))
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(20)
        marker.type = Marker.LINE_LIST
        marker.id = 0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 255/255.0
        marker.color.g = 255/255.0
        marker.color.b = 10/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for point in obstacles:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = z
            marker.points.append(p)
        self.pub_obstacles.publish(marker)

    def plot_boundary(self, boundary, z):
        # rospy.loginfo("boundary: {}".format(type(boundary)))
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(20)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 255/255.0
        marker.color.g = 255/255.0
        marker.color.b = 255/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for point in boundary:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = z
            marker.points.append(p)
        # marker.points.append(marker.points[0])
        self.pub_boudary.publish(marker)

    def plot_points(self, points, z):
        pts = points[:]
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(20)
        marker.type = Marker.SPHERE_LIST
        marker.id = 0
        marker.scale.x = 20
        marker.scale.y = 20
        marker.scale.z = 20
        marker.color.r = 10/255.0
        marker.color.g = 10/255.0
        marker.color.b = 255/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for point in pts:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = z
            marker.points.append(p)
        self.pub_points.publish(marker)
    
    def plot_path_optimized(self, polygon):
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(20)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.r = 155/255.0
        marker.color.g = 255/255.0
        marker.color.b = 155/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for point in polygon.points:
            p = Point()
            p.x = point.x
            p.y = point.y
            p.z = point.z
            marker.points.append(point)
        self.pub_path_optimized.publish(marker)
