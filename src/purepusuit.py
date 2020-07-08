#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial import distance
from geometry_msgs.msg import Pose
from menguiin.msg import erp42_write
from math import sin, cos, atan2


WHEEL_BASE = 1.04

class Pure_pursuit():
    def __init__(self):
        self.steer_pub = rospy.Publisher('erp42_write', erp42_write, queue_size=1)
        self.E_stop = 0
        self.gear = 0
        self.speed = 0
        self.steer = 0
        self.brake = 0
        self.sequence = 0
        self.heading = 0

        self.pure_pursuit()
    
    def position_listener(self):
        pose = rospy.wait_for_message('tmcoord', Pose)
        waypoint = np.load(file='/home/macaron/catkin_ws/src/menguiin/path/test.npy')
        next_goal =  waypoint[sequence+1]
        

        goal_dst = distance.euclidean(pose, next_goal)
        final_sequence = waypoint.shape[0]
        if goal_dst < 0.1:
            sequence += 1
        
        if sequence == final_sequence:
            print("Goal!")
            return -1

        return next_goal, goal_dst

    def xy2sl(self, x, y):
        heading = atan2(y, x)
        T = [[cos(heading), -sin(heading), x]\
             [sin(heading),  cos(heading), y]\
             [           0,             0, 1]]
        xy_pose = [x, y, 1]

        s, l = np.dot(np.linalg.inv(T), xy_pose.T)
        return s, l

    def pure_pursuit(self):
        erp = erp42_write()
        while not rospy.is_shutdown():
            next_goal, goal_dst = position_listener()
            s, l = xy2sl(next_goal[0], next_goal[1])

            heading = atan2(pose[1], pose[0])
            e_ld = next_goal[1] - l
            ld = goal_dst
            sine_alpha = e_ld/ld
            steer = atan2(2*WHEEL_BASE*sine_alpha, ld)

            erp.write_E_stop = 0
            erp.write_gear = 0
            erp.write_brake = 0
            erp.write_speed = 5
            erp.write_steer = steer

            steer_pub.publish(erp)


def main():
    rospy.init_node('pure_pursuit')
    try:
        Pure_pursuit()

    except rospy.ROSInterruptException:
        pass

## start code
if __name__ == '__main__':
    main()