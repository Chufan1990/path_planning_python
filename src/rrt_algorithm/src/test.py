#!/usr/bin/env python
import rospy
from rrt_interface import RRTInterface


def test():
    rospy.init_node('rrt_alogritm', anonymous=True)
    rospy.loginfo("="*len("ros init done"))
    rospy.loginfo("ros init done")
    rrt_problem = RRTInterface()
    rospy.loginfo("="*len("Interface init done"))
    rospy.loginfo("Interface init done")
    rrt_problem.update()
    # rrt_problem.test()


if __name__ == "__main__":
    try:
        test()
    except rospy.ROSInterruptException:
        pass
