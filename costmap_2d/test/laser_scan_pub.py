#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from sensor_msgs.msg import LaserScan

def talker():
    pub = rospy.Publisher('/internal/sensors/lidar/scan/filtered', LaserScan, queue_size=10)
    rospy.init_node('dummy_scanner', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    random.seed()
    seq = 0

    while not rospy.is_shutdown():

        scan = LaserScan()
        # Build the header
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = 'base_footprint'
        scan.header.seq = seq
        seq += 1

        # Build the scan
        scan.angle_min = -0.7
        scan.angle_max = 0.7
        scan.angle_increment = 0.7
        scan.time_increment = 0.01
        scan.scan_time = 0.01
        scan.range_min = 0.01
        scan.range_max = 10.0

        for k in range(int((scan.angle_max - scan.angle_min)/scan.angle_increment)):
            scan.ranges.append(1.5)# + random.random())

        pub.publish(scan)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass