#!/usr/bin/env python
import os
import rospkg
import rospy
import yaml
import SamplingBasedPathPlanningMethods
from environment import Environment
from RRT_family_planners import Node

def test():
	rospy.init_node("SBPP",anonymous=True)
	rospack = rospkg.RosPack()
	# data = yaml.load(open(os.path.join(rospack.get_path('SBPPlib'), 'config/hard_environment.yaml')))
	environment = Environment(os.path.join(rospack.get_path('sampling_based_path_planning'), 'config/simple.yaml'))
	bounds = (-2, -3, 6, 4)
	pose_start = (0, 0)
	pose_goal = (5, 3)
	object_radius = 0.3
	steer_distance = 0.3
	num_iterations = 5000
	resolution = 3
	visualization = True
	runForFullIterations = True

	tester = SamplingBasedPathPlanningMethods.SampleingBasedPathPlanner()
	tester.RRTStar( environment, bounds, pose_start, pose_goal, object_radius, steer_distance, num_iterations, resolution, runForFullIterations, visualization)

if __name__ == "__main__":
	try:
		test()
	except rospy.ROSInterruptException:
		pass
	