#!/usr/bin/env python
import rospy
from rrt_interface import RRTInterface


def test():
    rospy.init_node('test', anonymous=True)
    rospy.loginfo("ros init done")
    rrt_problem = RRTInterface()
    rospy.loginfo("Interface init done")
    rrt_problem.update()
    # rrt_problem.test()


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException:
        pass
