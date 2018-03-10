import time
import rospy
import numpy as numpy
import math
import geometry_msgs
from visualization_msgs.msg import Marker, MarkerArray

class RRTVisualizer:
    def __init__(self):
        # self.sub_way_point = rospy.Subscriber()

        self.pub_node_new = rospy.Publisher('/path_planning/viz/node_new', Marker, queue_size=1)
        self.pub_tree = rospy.Publisher('/path_planning/viz/tree', Marker, queue_size=1)
        self.pub_obstacles = rospy.Publisher('/path_planning/viz/obstacles', Marker, queue_size=1)
        self.pub_boudary = rospy.Publisher('/path_planning/viz/boundary', Marker, queue_size=1)
        

    def plot_node_new(self, way_pts, z_coord):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.3)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 100/255.0
        marker.color.r = 10/255.0
        marker.color.r = 10/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for way_pt in way_pts:
            p = geometry_msgs.msg.Point()
            p.x = way_pt[0]
            p.y = way_pt[1]
            p.z = z_coord
            marker.points.append(p)
        self.pub_node_new.publish(marker)

    def plot_tree(self, way_pts, z_coord):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.3)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 10/255.0
        marker.color.g = 100/255.0
        marker.color.b = 10/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for way_pt in way_pts:
            p = geometry_msgs.msg.Point()
            p.x = way_pt[0]
            p.y = way_pt[1]
            p.z = z_coord
            marker.points.append(p)
        self.pub_node_new.publish(marker)

    def plot_obstacles(self, way_pts, z_coord):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.3)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 10/255.0
        marker.color.r = 10/255.0
        marker.color.r = 10/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for way_pt in way_pts:
            p = geometry_msgs.msg.Point()
            p.x = way_pt[0]
            p.y = way_pt[1]
            p.z = z_coord
            marker.points.append(p)
        self.pub_node_new.publish(marker)

    def plot_boundary(self, way_pts, z_coord):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        marker.ns = "path_planning"
        marker.action = Marker.ADD
        marker.lifetime = rospy.Duration(0.3)
        marker.type = Marker.LINE_STRIP
        marker.id = 0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 40/255.0
        marker.color.r = 40/255.0
        marker.color.r = 40/255.0
        marker.color.a = 1.0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        for way_pt in way_pts:
            p = geometry_msgs.msg.Point()
            p.x = way_pt[0]
            p.y = way_pt[1]
            p.z = z_coord
            marker.points.append(p)
        self.pub_node_new.publish(marker)