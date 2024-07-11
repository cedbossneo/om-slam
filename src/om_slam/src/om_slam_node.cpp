#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <xbot_msgs/AbsolutePose.h>

// Global variables
laser_geometry::LaserProjection* laser_projector_;
ros::Publisher pc_pub_;
ros::Publisher absolute_pose_pub_;

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    sensor_msgs::PointCloud2 cloud_msg;
    laser_projector_->projectLaser(*scan_msg, cloud_msg);
    pc_pub_.publish(cloud_msg);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose_stamped) {
    xbot_msgs::AbsolutePose xb_absolute_pose_msg;
    xb_absolute_pose_msg.header = pose_stamped->header;
    xb_absolute_pose_msg.sensor_stamp = 0;
    xb_absolute_pose_msg.received_stamp = 0;
    xb_absolute_pose_msg.source = xbot_msgs::AbsolutePose::SOURCE_SENSOR_FUSION;
    xb_absolute_pose_msg.flags = xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_DEAD_RECKONING;
    xb_absolute_pose_msg.orientation_valid = true;
    xb_absolute_pose_msg.motion_vector_valid = false;

    xb_absolute_pose_msg.position_accuracy = 0.01;

    xb_absolute_pose_msg.flags |= xbot_msgs::AbsolutePose::FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE;

    xb_absolute_pose_msg.orientation_accuracy = 0.01;
    geometry_msgs::PoseWithCovariance pose_with_covariance;
    pose_with_covariance.pose = pose_stamped->pose;
    xb_absolute_pose_msg.pose = pose_with_covariance;
    xb_absolute_pose_msg.vehicle_heading = pose_stamped->pose.orientation.z;
    xb_absolute_pose_msg.motion_heading = pose_stamped->pose.orientation.z;

    absolute_pose_pub_.publish(xb_absolute_pose_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "om_slam_node");

    ros::NodeHandle n;

    // Initialize laser projector
    laser_projector_ = new laser_geometry::LaserProjection();

    // Set up subscribers and publishers
    pc_pub_ = n.advertise<sensor_msgs::PointCloud2>("/points2", 10);
    absolute_pose_pub_ = n.advertise<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", 50);

    ros::Subscriber scan_sub_ = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
    ros::Subscriber pose_sub_ = n.subscribe<geometry_msgs::PoseStamped>("/tracked_pose", 50, poseCallback);

    ROS_INFO("om_slam_node initialized.");

    ros::spin();
    ROS_INFO("om_slam_node uninitialized.");

    // Cleanup
    delete laser_projector_;

    return 0;
}