import rospy
from RRT_family_planners import RRTFamilyPathPlanner
from ROS_Visualizor import RRTFamilyVisualizer
from path_planning_msgs.msg import Line, LineArray


class SampleingBasedPathPlanner(object):

    def __init__(self):
        self.RRTFamilySolver = RRTFamilyPathPlanner()
        self.RRTFamilyViz = RRTFamilyVisualizer()

    def RRT(self, environment, bounds, pose_start, pose_goal, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, visualization=True):
        """ Returns path connecting start point to goal region in the configuration space using  RRT algorithm

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
                RRT_Flavour				( string ):						String representing the algorithm expected. Optian: "RRT", "RRT*", "InformedRRT*". Anything else returns None

        Returns:
                path 					( list<float, float> ):			A list of tuple/coordinates representing the nodes in path connecting start point and goal region
                self.tree 				( list<Node> ):					A list of nodes in the tree
        """
        time_start = rospy.get_rostime()

        path, vertex = self.RRTFamilySolver.update(environment, bounds, pose_start, pose_goal, object_radius,
                                                 steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT")

        time_elapsed = rospy.get_rostime() - time_start
        rospy.loginfo("cost: {} secs".format(time_elapsed*1e-9))

        if visualization:
            self.RRTFamilyViz.plot(environment, bounds, pose_start, pose_goal, object_radius, resolution, vertex, path)

    def RRTStar(self, environment, bounds, pose_start, pose_goal, object_radius, steer_distance, num_iterations, resolution=3, runForFullIterations=False, visualization=True):
        """ Returns path connecting start point to goal region in the configuration space using RRT* algorithm2

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
                RRT_Flavour				( string ):						String representing the algorithm expected. Optian: "RRT", "RRT*", "InformedRRT*". Anything else returns None

        Returns:
                path 					( list<float, float> ):			A list of tuple/coordinates representing the nodes in path connecting start point and goal region
                tree 					( list<Node> ):					A list of nodes in the tree
        """
        time_start = rospy.get_rostime()

        path, vertex = self.RRTFamilySolver.update(environment, bounds, pose_start, pose_goal, object_radius,
                                                 steer_distance, num_iterations, resolution, runForFullIterations, RRT_Flavour="RRT*")
        
        time_elapsed = rospy.get_rostime() - time_start
        rospy.loginfo("cost: {} secs".format(time_elapsed*1e-9))

        if visualization:
            self.RRTFamilyViz.plot(environment, bounds, pose_start, pose_goal, object_radius, resolution, vertex, path) 

    def InformedRRTStar(self, environment, bounds, start_pose, goal_region, object_radius, steer_distance, num_iterations, resolution=16, runForFullIterations=False):
        pass
