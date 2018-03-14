#!/usr/bin/env python
import rospy
from RRT_family_planners import Node
import shapely.geometry
from visualization_msgs.msg import Marker, MarkerArray
import geometry_msgs.msg

class RRTFamilyVisualizer:
	def __init__(self):
		self.pub_tree = rospy.Publisher(
			'/path_planning/viz/tree', Marker, queue_size=1)
		self.pub_obstacles = rospy.Publisher(
			'/path_planning/viz/obstacles', Marker, queue_size=1)
		self.pub_path = rospy.Publisher(
			'/path_planning/viz/path', Marker, queue_size=1)
		self.pub_boudary = rospy.Publisher(
			'/path_planning/viz/boundary', Marker, queue_size=1)
		self.pub_point = rospy.Publisher(
			'/path_planning/viz/start_point', Marker, queue_size=1)
		self.pub_region = rospy.Publisher(
			'/path_planning/viz/goal_region', Marker, queue_size=1)

	def initializer(self, environment, bounds, pose_start, pose_goal, object_radius, resolution, tree, path):
		self.obstacles = environment.obstacles
		self.bounds = bounds
		self.pose_start = pose_start
		self.goal_region = list(shapely.geometry.Point(pose_goal).buffer(object_radius, resolution).exterior.coords)
		self.tree = tree
		self.path = path

	def plot(self, environment, bounds, pose_start, pose_goal, object_radius, resolution, tree, path):
		self.initializer(environment, bounds, pose_start, pose_goal, object_radius, resolution, tree, path)
		self.parser_boundary()
		self.parser_obstacles()
		self.parser_point()
		self.parser_region()
		self.parser_tree()
		self.parser_path()

  	def parser_tree(self):
  		plot_tree = list()
  		for node in self.tree:
			if node.parent != None:
				plot_tree.append(node.parent.point)
				plot_tree.append(node.point)
		self.plot_tree(plot_tree)

  	def parser_path(self):
  		plot_path = list()
  		for point in self.path:
  			plot_path.append(point)
  			self.plot_path(plot_path)
  			# self.rate.sleep()

	def parser_obstacles(self):
		o = self.obstacles
		hmi_x = []
		hmi_y = []
		for obs in o:
			corners = list(obs.exterior.coords)
			for i in range(0, len(corners)-1):
				hmi_x += [corners[i][0], corners[i+1][0]]
				hmi_y += [corners[i][1], corners[i+1][1]]
		obstacles = zip(hmi_x, hmi_y)
		self.plot_obstacles(obstacles)

	def parser_boundary(self):
		x_min, y_min, x_max, y_max = self.bounds
		boundary = [(x_min, y_min), (x_min, y_max), (x_max, y_max), (x_max, y_min), (x_min, y_min)]
		self.plot_boundary(boundary)

	def parser_point(self):
		self.plot_point(self.pose_start)

	def parser_region(self):
		self.plot_region(self.goal_region)

	def plot_tree(self, tree):
		marker = Marker()
		marker.header.frame_id = 'path_planning'
		marker.header.stamp = rospy.Time.now()
		marker.ns = "path_planning"
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(300)
		marker.type = Marker.LINE_LIST
		marker.id = 0
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.r = 110/255.0
		marker.color.g = 110/255.0
		marker.color.b = 110/255.0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		for point in tree:
			p = geometry_msgs.msg.Point()
			p.x = point[0]
			p.y = point[1]
			p.z = 0.0
			marker.points.append(p)
		self.pub_tree.publish(marker)

	def plot_path(self, path):
		marker = Marker()
		marker.header.frame_id = 'path_planning'
		marker.header.stamp = rospy.Time.now()
		marker.ns = "path_planning"
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(300)
		marker.type = Marker.LINE_STRIP
		marker.id = 0
		marker.scale.x = 0.02
		marker.scale.y = 0.02
		marker.scale.z = 0.02
		marker.color.r = 255/255.0
		marker.color.g = 10/255.0
		marker.color.b = 10/255.0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		for point in path:
			p = geometry_msgs.msg.Point()
			p.x = point[0]
			p.y = point[1]
			p.z = 0
			marker.points.append(p)
		self.pub_path.publish(marker)

	def plot_obstacles(self, obstacles):
		marker = Marker()
		marker.header.frame_id = 'path_planning'
		marker.header.stamp = rospy.Time.now()
		marker.ns = "path_planning"
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(300)
		marker.type = Marker.LINE_LIST
		marker.id = 0
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.r = 255/255.0
		marker.color.g = 255/255.0
		marker.color.b = 10/255.0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		for point in obstacles:
			p = geometry_msgs.msg.Point()
			p.x = point[0]
			p.y = point[1]
			p.z = 0
			marker.points.append(p)
		self.pub_obstacles.publish(marker)

	def plot_boundary(self, boundary):
		marker = Marker()
		marker.header.frame_id = 'path_planning'
		marker.header.stamp = rospy.Time.now()
		marker.ns = "path_planning"
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(300)
		marker.type = Marker.LINE_STRIP
		marker.id = 0
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.r = 255/255.0
		marker.color.g = 255/255.0
		marker.color.b = 255/255.0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		for point in boundary:
			p = geometry_msgs.msg.Point()
			p.x = point[0]
			p.y = point[1]
			p.z = 0
			marker.points.append(p)
		self.pub_boudary.publish(marker)

	def plot_point(self, point):
		marker = Marker()
		marker.header.frame_id = 'path_planning'
		marker.header.stamp = rospy.Time.now()
		marker.ns = "path_planning"
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(300)
		marker.type = Marker.SPHERE_LIST
		marker.id = 0
		marker.scale.x = 0.1
		marker.scale.y = 0.1
		marker.scale.z = 0.1
		marker.color.r = 10/255.0
		marker.color.g = 10/255.0
		marker.color.b = 255/255.0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		p = geometry_msgs.msg.Point()
		p.x = point[0]
		p.y = point[1]
		p.z = 0
		marker.points.append(p)
		self.pub_point.publish(marker)

	def plot_region(self, region):
		marker = Marker()
		marker.header.frame_id = 'path_planning'
		marker.header.stamp = rospy.Time.now()
		marker.ns = "path_planning"
		marker.action = Marker.ADD
		marker.lifetime = rospy.Duration(300)
		marker.type = Marker.LINE_STRIP
		marker.id = 0
		marker.scale.x = 0.01
		marker.scale.y = 0.01
		marker.scale.z = 0.01
		marker.color.r = 155/255.0
		marker.color.g = 255/255.0
		marker.color.b = 155/255.0
		marker.color.a = 1.0
		marker.pose.orientation.x = 0
		marker.pose.orientation.y = 0
		marker.pose.orientation.z = 0
		marker.pose.orientation.w = 1
		for point in region:
			p = geometry_msgs.msg.Point()
			p.x = point[0]
			p.y = point[1]
			p.z = 0
			marker.points.append(p)
		self.pub_region.publish(marker)
