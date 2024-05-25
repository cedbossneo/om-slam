#!/usr/bin/env python3

import rospy
import tf2_ros
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseStamped

class OdomToTF:
    def __init__(self):
        rospy.init_node('odom_to_tf', anonymous=True)

        self.br = tf2_ros.TransformBroadcaster()
        self.pose_pub = rospy.Publisher('/slam_toolbox/pose', PoseStamped, queue_size=10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Parameters
        self.map_frame = rospy.get_param('~map_frame', 'map_laser')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_link_frame = rospy.get_param('~base_link_frame', 'base_laser')
        # Subscriber
        rospy.Subscriber('/xbot_positioning/odom_out', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        current_time = rospy.Time.now()

        # Publish map -> odom transform
        odom_trans = TransformStamped()
        odom_trans.header.stamp = current_time
        odom_trans.header.frame_id = self.odom_frame
        odom_trans.child_frame_id = self.base_link_frame
        odom_trans.transform.translation.x = msg.pose.pose.position.x
        odom_trans.transform.translation.y = msg.pose.pose.position.y
        odom_trans.transform.translation.z = msg.pose.pose.position.z
        odom_trans.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(odom_trans)

        #rospy.loginfo(f"Published transforms and pose at time {current_time.to_sec()}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = OdomToTF()
        node.run()
    except rospy.ROSInterruptException:
        pass