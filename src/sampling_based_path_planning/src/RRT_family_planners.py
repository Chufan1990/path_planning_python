from __future__ import division
from shapely.geometry import Point, LineString
import random
import math
import numpy as np
import rospy
from geometry_msgs.msg import Point32

class RRTFamilyPathPlanner(object):

	""" Rapidly-exploring Random tree Family.

	contains methods for simple RRT-based search, RRT* based search.

	TODO: informed RRT* based search and many other optimization.

	"""

	def initializer(self, environment, bounds, pose_start, pose_goal, object_radius, steer_distance, num_iteration, resolution, runForFullIterations):

		""" initialize the planner with information about the environment and parameters for the RRT path planner.

		Args:
			environment 			(a yaml environment): 			Configuration space including obstacles.
			bounds 					( (float float float float) ): 	x min, y min, x max and y max, coordinates fo the bounds of the configuration space.
			pose_start				( (float float) ):  			Position of the initial point in configuration space, specified in problem.
			pose_goal				( (float, float) ):				Position of the target point in configuration space, speciied in problem.
			object_radius 			( float ):						Radius of objects.
			steer_distance  		( float ):						Limit of the length of branch in tree.
			num_iterations			( int ):						Max sample times for creating tree.
			resolution				( int ):						Number of segments used to approximate a quarter circle arount point.
			runForFullIterations	( boolean ):					If True RRT and RRTStar return the first path found without having to sample all num_iterations points. Default value = False.

		Returns:
			None
		"""

		# environment
		self.env = environment
		self.obstacles = environment.obstacles
		self.bounds = bounds
		self.x_min, self.y_min, self.x_max, self.y_max = bounds

		# start and goal
		self.pose_start = pose_start
		self.pose_goal = pose_goal
		self.node_start = Node(pose_start, None)
		self.node_goal = Node(pose_goal, None)

		# constant
		self.object_radius = object_radius
		self.num_iteration = num_iteration
		self.resolution = resolution
		self.steer_distance = steer_distance

		self.vertex = set()
		self.path = list()
		# boolean
		self.runForFullIterations = runForFullIterations

	def update(self, environment, bounds, pose_start, pose_goal, object_radius, steer_distance, num_iteration, resolution, runForFullIterations, RRT_Flavour):
		""" Returns path connecting start point to goal region in the configuration space using specified RRT-variant algorithm

		Args:
			environment 			( a yaml environment ): 		Configuration space including obstacles.
			bounds 					( (float float float float) ): 	x min, y min, x max and y max, coordinates fo the bounds of the configuration space.
			pose_start				( (float float) ):  			Position of the initial point in configuration space, specified in problem.
			pose_goal				( (float, float) ):				Position of the target point in configuration space, speciied in problem.
			object_radius 			( float ):						Radius of objects.
			steer_distance  		( float ):						Limit of the length of branch in tree.
			num_iterations			( int ):						Max sample times for creating tree.
			resolution				( int ):						Number of segments used to approximate a quarter circle arount point.
			runForFullIterations	( boolean ):					If True RRT and RRTStar return the first path found without having to sample all num_iterations points. Default value = False.
			RRT_Flavour				( string ):						String representing the algorithm expected. Optian: "RRT", "RRT*", "InformedRRT*". Anything else returns None
		
		Returns:
			path 					( list<float, float> ):			A list of tuple/coordinates representing the nodes in path connecting start point and goal region
			self.vertex 			( set<Node> ):					A list of nodes in the tree
		"""
		
		self.env = environment
		self.initializer(environment, bounds, pose_start, pose_goal, object_radius, steer_distance, num_iteration, resolution, runForFullIterations)

		node_start, node_goal = self.node_start, self.node_goal
		if node_start.point == node_goal.point:
			self.vertex.add(node_start)
			self.vertex.add(node_goal)
			self.path.append(node_start.point, node_goal.point)
		elif self.isEdgeCollisionFree(node_start.point, node_goal.point):
			node_goal.parent = node_start
			self.vertex.add(node_start)
			self.vertex.add(node_goal)
			self.path.append(node_start.point, node_goal.point)
		else:
			if RRT_Flavour == "RRT":
				self.path, self.vertex = self.RRTSearch()
			elif RRT_Flavour == "RRT*":
				self.path, self.vertex = self.RRTStarSearch()
			elif RRT_Flavour == "InformedRRT*":
				pass
			else:
				self.path = []
		return self.path, self.vertex


	def RRTSearch(self):
		"""Returns path using RRT algorithm.

		Builds a tree exploring from the start node until it reaches the goal region. It works by sampling random points in the map and connecting them with
		the tree we build off on each iteration of the algorithm.

		Returns:
			path 					( list<(float, float)> ): 		A list of tuples/coordinates representing the nodes in a path from start to the goal region
			self.vertex				( set<Node> ): 					A list of Node (coordinates and parent node) of nodes in the tree
		"""
		path = list()
		path_length = float("inf")
		node_start = self.node_start
		self.vertex.add(node_start)
		for i in range(self.num_iteration):
			rospy.loginfo("iteration: {}".format(i))
			point_random = self.get_random_clean()
			node_nearest = self.find_node_nearest(point_random)
			point_new = self.steer(node_nearest.point, point_random)

			if self.isEdgeCollisionFree(node_nearest.point, point_new):
				### For RRT*
				# set_nearest = self.find_set_nearest(point_new)
				# node_min = self.find_node_min(set_nearest, node_nearest, point_new)  
				# node_new = Node(point_new, node_min)
				###
				
				### For RRT
				node_new = Node(point_new, node_nearest)  # delete if RRT*
				###

				self.vertex.add(node_new)
				### For RRT*
				# self.rewire()
				###
				if self.isAtGoalRegion(point_new):
					if not self.runForFullIterations:
						path = self.find_path(node_new, node_start)
						path = self.find_path_smooth(path)
						break
					else:
						path_temp = self.find_path(node_new, node_start)
						path_temp = self.find_path_smooth(path_temp)
						if len(path_temp) < path_length:
							path_length = len(path_temp)
							path = path_temp
		return path, self.vertex





	def RRTStarSearch(self):
		"""Returns path using RRT* algorithm.

		Builds a tree exploring from the start node until it reaches the goal region. It works by sampling random points in the map and connecting them with
		the tree we build off on each iteration of the algorithm.

		Returns:
			path 					( list<(float, float)> ): 		A list of tuples/coordinates representing the nodes in a path from start to the goal region
			self.vertex 			( set<Node> ): 					A list of Node (coordinates and parent node) of nodes in the tree
		"""
		path = list()
		path_length_tmp = float("inf")
		node_start = self.node_start
		self.vertex.add(node_start)
		for i in range(self.num_iteration):
			rospy.loginfo("iteration: {}".format(i))
			point_random = self.get_random_clean()
			node_nearest = self.find_node_nearest(point_random)
			point_new = self.steer(node_nearest.point, point_random)
			if self.isEdgeCollisionFree(node_nearest.point, point_new):
				### For RRT*
				set_nearest = self.find_set_nearest(point_new)
				node_min = self.find_node_min(set_nearest, node_nearest, point_new)  
				node_new = Node(point_new, node_min)
				############

				### For RRT
				# node_new = Node(point_new, node_nearest)  # delete if RRT*
				###########
				self.vertex.add(node_new)
				self.rewire(set_nearest, node_new, node_min)
				if self.isAtGoalRegion(point_new):
					if not self.runForFullIterations:
						path, __ = self.find_path(node_new, node_start)
						path = self.find_path_smooth(path)
						break
					else:
						path_temp, path_length = self.find_path(node_new, node_start)
						# path_temp = self.find_path_smooth(path_temp)
						if path_length < path_length_tmp:
							path_length_tmp = path_length
							path = path_temp
							path_temp = self.find_path_smooth(path_temp)
		return path, self.vertex

	def InformedRRTStarSearch(self):
		pass

	def get_random(self):
		# return a point within the bounds
		x = random.uniform(self.x_min, self.x_max)
		y = random.uniform(self.y_min, self.y_max)
		return (x ,y)

	def get_random_clean(self):
		# Run until a valid point is found
		while True:
			point = self.get_random()
			# Pick a point and return if no obstacle overlaps with a circle centred at it
			buffer_of_point = Point(point).buffer(self.object_radius, self.resolution)
			if self.isPointCollisionFree(buffer_of_point):
				return point

	def isOutOfBounds(self, point):
		if((point[0] - self.object_radius) < self.x_min):
			return True
		if((point[1] - self.object_radius) < self.y_min):
			return True
		if((point[0] + self.object_radius) > self.x_max):
			return True
		if((point[1] + self.object_radius) > self.y_max):
			return True
		return False

	def isPointCollisionFree(self, point):
		for obstacle in self.obstacles:
			if obstacle.contains(point):
				return False
		return True

	def isEdgeCollisionFree(self, point_from, point_to):
		if self.isOutOfBounds(point_to):
			return False
		line = LineString([point_from, point_to])
		line_expanded = line.buffer(self.object_radius, self.resolution)
		for obstacle in self.obstacles:
			if line_expanded.intersects(obstacle):
				return False
		return True

	def isAtGoalRegion(self, point_new):
		point_goal = Point(self.pose_goal)
		point_new = Point(point_new)
		goal_region = point_goal.buffer(self.object_radius, self.resolution)
		if goal_region.contains(point_new):
			return True
		return False

	def find_node_nearest(self, point_random):
		node_nearest = Node(None, None)
		dist_min = float("inf")
		for node in self.vertex:
			dist = self.euclidian_dist(point_random, node.point)
			if dist < dist_min:
				dist_min = dist
				node_nearest = node
		return node_nearest

	def find_set_nearest(self, point_new):
		nodes = set()
		ball_radius = self.find_ball_radius()
		for node in self.vertex:
			if self.euclidian_dist(node.point, point_new) < ball_radius:
				nodes.add(node)
		return nodes

	def find_node_min(self, set_nearest, node_nearest, point_new):
		node_min = node_nearest
		cost_min = self.cost(node_nearest) + self.linecost(node_nearest.point, point_new)
		for node in set_nearest:
			if self.isEdgeCollisionFree(node.point, point_new):
				cost_temp = self.cost(node) + self.linecost(node.point, point_new)
				if cost_temp < cost_min:
					node_min = node
					cost_min = cost_temp
		return node_min

	def find_ball_radius(self):
		unit_ball_volume = math.pi
		n = len(self.vertex)
		dimension = 2.0
		gamma = (1 + 1/dimension) * (self.x_max - self.x_min) * (self.y_max - self.y_min) / unit_ball_volume
		ball_radius = min(2*(gamma * math.log(n)/n)**(1/dimension), self.steer_distance)
		return ball_radius

	def find_path(self, node_from, node_to):
		# return path containing coordinates of every node between(including) node_from and node_to 
		path = list()
		path_length = 0
		node = node_from
		while node.point != node_to.point:
			path_length += self.euclidian_dist(node.point, node.parent.point)
			path.append(node.point)
			node = node.parent
		path.append(node_to.point)
		path.reverse()
		return path, path_length

	def find_path_smooth(self, path):
		point_end = path[-1]
		point_start = path[0]
		path_smooth = list()
		path_smooth.append(point_end)
		while point_end != point_start:
			for point in path:
				if self.isEdgeCollisionFree(point, point_end):
					path_smooth.append(point)
					point_end = point
					break
		return path_smooth

	def cost(self, node_from):
	 	__, path_length = self.find_path(node_from, self.node_start)
	 	return path_length

	def linecost(self, point_from, point_to):
	 	return self.euclidian_dist(point_from, point_to)


	def euclidian_dist(self, point_from, point_to):
	 	if point_from != None and point_to != None:
	 		return math.hypot(point_to[1]-point_from[1], point_to[0]-point_from[0])
	 	else:
	 		return float("inf")

	def steer(self, point_from, point_to):
		if self.euclidian_dist(point_from ,point_to) < self.steer_distance:
			return point_to
		else:
			x_from, y_from = point_from
			x_to, y_to = point_to
			theta = math.atan2(y_to - y_from, x_to - x_from)
			point_new = (x_from + self.steer_distance * math.cos(theta), y_from + self.steer_distance * math.sin(theta))
			return point_new

	def rewire(self, set_nearest, node_new, node_min):
		for node in set_nearest-set([node_min]):
			if self.isEdgeCollisionFree(node.point, node_new.point):
				if self.cost(node) > self.cost(node_new) + self.linecost(node.point, node_new.point):
					self.vertex.discard(node)
					node_rewired = Node(node.point, node_new)
					self.vertex.add(node_rewired)

class Node(object):
	""" 
	Node structrue for RRT family path planner

	Args:
		point: 						( (float, float) )				Coordinates of current node, datatype: tuple, Point
		parent: 					( Node ):						Parent node of current node, datetype: Node
	"""
	def __init__(self, point, parent):
		self.point = point
		self.parent = parent
