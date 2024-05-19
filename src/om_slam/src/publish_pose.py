#!/usr/bin/env python3
import rospy
import tf
from geometry_msgs.msg import PoseStamped

def publish_pose():
    rospy.init_node('pose_publisher', anonymous=True)
    pose_publisher = rospy.Publisher('/slam_out_pose', PoseStamped, queue_size=10)
    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "map"
            pose_msg.pose.position.x = trans[0]
            pose_msg.pose.position.y = trans[1]
            pose_msg.pose.position.z = trans[2]
            pose_msg.pose.orientation.x = rot[0]
            pose_msg.pose.orientation.y = rot[1]
            pose_msg.pose.orientation.z = rot[2]
            pose_msg.pose.orientation.w = rot[3]

            pose_publisher.publish(pose_msg)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_pose()
    except rospy.ROSInterruptException:
        pass