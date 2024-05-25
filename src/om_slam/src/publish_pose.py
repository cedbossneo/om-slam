#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovariance, Vector3
from xbot_msgs.msg import AbsolutePose
from tf import transformations

class PoseConverter:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pose_converter', anonymous=True)

        # Subscriber to the PoseStamped topic
        self.pose_sub = rospy.Subscriber('/tracked_pose', PoseStamped, self.pose_callback)

        # Publisher to the AbsolutePose topic
        self.pose_pub = rospy.Publisher('/xbot_positioning/xb_pose_out', AbsolutePose, queue_size=10)

    def pose_callback(self, msg):
        # Create an instance of AbsolutePose
        absolute_pose = AbsolutePose()

        # Populate the AbsolutePose message
        absolute_pose.header = msg.header
        absolute_pose.sensor_stamp = 0  # Example timestamp, replace with actual sensor timestamp if available
        absolute_pose.received_stamp = 0
        absolute_pose.source = AbsolutePose.SOURCE_SENSOR_FUSION  # Example source, modify as needed
        absolute_pose.flags = AbsolutePose.FLAG_SENSOR_FUSION_DEAD_RECKONING
        absolute_pose.orientation_valid = 1
        absolute_pose.motion_vector_valid = 0
        absolute_pose.position_accuracy = 0.02  # Example accuracy, replace with actual data if available
        absolute_pose.orientation_accuracy = 0.01  # Example accuracy, replace with actual data if available

        # Fill in the pose with covariance
        absolute_pose.pose.pose = msg.pose
        absolute_pose.pose.covariance = [0] * 36  # Example covariance, replace with actual data if available

        # Example motion vector, replace with actual data if available
        absolute_pose.motion_vector = Vector3(0.0, 0.0, 0.0)

        # Example headings, replace with actual data if available
        absolute_pose.vehicle_heading = self.get_vehicle_heading(msg)
        absolute_pose.motion_heading = absolute_pose.vehicle_heading

        rospy.loginfo("Publishing AbsolutePose message.")
        # Publish the AbsolutePose message
        self.pose_pub.publish(absolute_pose)

    def get_vehicle_heading(self, msg):
        # Replace `x` with the actual object or computation to get theta
        # Assuming `msg` contains the necessary data to compute theta
        # Example calculation, replace with actual logic
        theta = self.calculate_theta_from_pose(msg.pose)
        return theta

    def calculate_theta_from_pose(self, pose):
        # Placeholder for actual calculation
        # For example, calculating yaw from quaternion in pose.orientation
        orientation_q = pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = transformations.euler_from_quaternion(orientation_list)
        return yaw

if __name__ == '__main__':
    try:
        converter = PoseConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass