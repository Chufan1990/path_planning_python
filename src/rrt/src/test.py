import numpy as np
import random
from hmi_publisher import *


def way_point_generator():
    rospy.init_node('way_point_generator', anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rand = random.random()
        yield zip(np.linspace(rand, rand+100, 100), np.linspace(rand, rand+100, 100),np.linspace(rand, rand+100, 100))
        # yield zip([[1,100],[200,1000]],[[1,100],[200,1000]])
        rate.sleep()

if __name__  =="__main__":
    a = WayPointVisualizer()
    for i in way_point_generator():
        a.plot_node_new(i,0)
        
