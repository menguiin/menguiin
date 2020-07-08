#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import distance

iteration = 0
waypoint = np.empty((1,2))

class Draw_map():
    def __init__(self):
        self.listen_xy = rospy.Subscriber('tmcoord', Pose, self.pose_callback, queue_size=1)
        self.listen()

    def pose_callback(self, pose):
        global iteration, waypoint
        
        waypoint = np.append(waypoint, np.array([[pose.position.x, pose.position.y]]), axis=0)

        iteration += 1
        if iteration > 5: # 5 seconds after
            dst = distance.euclidean(waypoint[1], waypoint[-1])
            print(waypoint[1])
            print(waypoint[-1])
            print(dst)
            if dst < 0.5: 
                waypoint = np.delete(waypoint[0])
                np.save('/home/macaron/catkin_ws/src/menguiin/path/test', waypoint)
                print('done!')
                plt.plot(waypoint[:, 0], waypoint[:, 1])
                plt.show()
                return -1

    
    def listen(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


def main():
    rospy.init_node('Mapping')
    try:
        Draw_map()

    except rospy.ROSInterruptException:
        pass

## start code
if __name__ == '__main__':
    main()
