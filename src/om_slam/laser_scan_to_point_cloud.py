#!/usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
import laser_geometry.laser_geometry as lg

class LaserScanToPointCloud:
    def __init__(self):
        self.laser_projector = lg.LaserProjection()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.pc_pub = rospy.Publisher('/points2', PointCloud2, queue_size=1)

    def scan_callback(self, scan_msg):
        # Convert LaserScan to PointCloud2
        cloud_msg = self.laser_projector.projectLaser(scan_msg)
        # Publish the PointCloud2
        self.pc_pub.publish(cloud_msg)

if __name__ == '__main__':
    rospy.init_node('laser_scan_to_point_cloud')
    converter = LaserScanToPointCloud()
    rospy.spin()