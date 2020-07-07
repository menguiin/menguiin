#!/usr/bin/env python

import rospy
from pyproj import Proj, transform
from geometry_msgs.msg import Pose
from sensor_msgs.msg import NavSatFix

#Projection definition
#UTM-K
proj_UTMK = Proj(init='epsg:5178')

#WGS1984
proj_WGS84 = Proj(init='epsg:4326')

class tm_receive():
    def __init__(self):
        self.tm_coord = rospy.Publisher('tmcoord', Pose, queue_size=1)
        self.wgs2utm()

    def wgs_listener(self):
        wgs = rospy.wait_for_message('ublox_gps/fix', NavSatFix)
        lat = wgs.latitude
        lon = wgs.longitude

        return lon, lat

    def wgs2utm(self):
        pose = Pose()
        while not rospy.is_shutdown():
            lon, lat = self.wgs_listener()
            x, y = transform(proj_WGS84, proj_UTMK, lon, lat)
            pose.position.x = x
            pose.position.y = y

            print(lon, lat)
            print(x, y)
            self.tm_coord.publish(pose)

def main():
    rospy.init_node('GPS_reader')
    try:
        tm_receive()

    except rospy.ROSInterruptException:
        pass


## start code
if __name__ == '__main__':
    main()