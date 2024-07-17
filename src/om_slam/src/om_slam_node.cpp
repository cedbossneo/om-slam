#include <ros/ros.h>
#include <ros/topic.h> // Include the header for waitForMessage
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <laser_geometry/laser_geometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <xbot_msgs/AbsolutePose.h>
#include <xbot_msgs/WheelTick.h>
#include "xbot_positioning_core.h"
#include "xbot_positioning/GPSControlSrv.h"
#include "xbot_positioning/SetPoseSrv.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Global variables
laser_geometry::LaserProjection* laser_projector_;
ros::Publisher pc_pub_;
ros::Publisher absolute_pose_pub_;
ros::Publisher odometry_pub;
nav_msgs::Odometry odometry;
bool has_ticks;
xbot_msgs::WheelTick last_ticks;
bool has_gyro;
bool has_gps;
sensor_msgs::Imu last_imu;
ros::Time gyro_calibration_start;
double gyro_offset;
int gyro_offset_samples;
// Current speed calculated by wheel ticks
double vx = 0.0;
bool skip_gyro_calibration;

xbot::positioning::xbot_positioning_core core{};

void onImu(const sensor_msgs::Imu::ConstPtr &msg) {
    if(!has_gyro) {
        if(!skip_gyro_calibration) {
            if (gyro_offset_samples == 0) {
                ROS_INFO_STREAM("Started gyro calibration");
                gyro_calibration_start = msg->header.stamp;
                gyro_offset = 0;
            }
            gyro_offset += msg->angular_velocity.z;
            gyro_offset_samples++;
            if ((msg->header.stamp - gyro_calibration_start).toSec() < 5) {
                last_imu = *msg;
                return;
            }
            has_gyro = true;
            if (gyro_offset_samples > 0) {
                gyro_offset /= gyro_offset_samples;
            } else {
                gyro_offset = 0;
            }
            gyro_offset_samples = 0;
            ROS_INFO_STREAM("Calibrated gyro offset: " << gyro_offset);
        } else {
            ROS_WARN("Skipped gyro calibration");
            has_gyro = true;
            return;
        }
    }
    if (!has_gps) {
        return;
    }
    core.predict(vx, msg->angular_velocity.z - gyro_offset, (msg->header.stamp - last_imu.header.stamp).toSec());
    auto x = core.updateSpeed(vx, msg->angular_velocity.z - gyro_offset,0.01);

    odometry.header.stamp = ros::Time::now();
    odometry.header.seq++;
    odometry.header.frame_id = "map";
    odometry.child_frame_id = "base_link";
    odometry.pose.pose.position.x = x.x_pos();
    odometry.pose.pose.position.y = x.y_pos();
    odometry.pose.pose.position.z = 0;
    tf2::Quaternion q(0.0, 0.0, x.theta());
    odometry.pose.pose.orientation = tf2::toMsg(q);

    odometry_pub.publish(odometry);
    last_imu = *msg;
}

void onWheelTicks(const xbot_msgs::WheelTick::ConstPtr &msg) {
    if(!has_ticks) {
        last_ticks = *msg;
        has_ticks = true;
        return;
    }
    double dt = (msg->stamp - last_ticks.stamp).toSec();

    double d_wheel_l = (double) (msg->wheel_ticks_rl - last_ticks.wheel_ticks_rl) * (1/(double)msg->wheel_tick_factor);
    double d_wheel_r = (double) (msg->wheel_ticks_rr - last_ticks.wheel_ticks_rr) * (1/(double)msg->wheel_tick_factor);

    if(msg->wheel_direction_rl) {
        d_wheel_l *= -1.0;
    }
    if(msg->wheel_direction_rr) {
        d_wheel_r *= -1.0;
    }


    double d_ticks = (d_wheel_l + d_wheel_r) / 2.0;
    vx = d_ticks / dt;
    // Ignore implausible wheel tick calculation
    if(abs(vx) > 0.6) {
        ROS_WARN_STREAM("got vx > 0.6 (" << vx << ") - dropping measurement");
        vx = 0.0;
        return;
    }

    last_ticks = *msg;
}

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

void onGpsPose(const xbot_msgs::AbsolutePose::ConstPtr &msg) {
    if((msg->flags & (xbot_msgs::AbsolutePose::FLAG_GPS_RTK_FIXED)) == 0) {
        ROS_INFO_STREAM_THROTTLE(1, "Dropped GPS update, since it's not RTK Fixed");
        core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 500);
        has_gps = true;
        return;
    }
    core.updatePosition(msg->pose.pose.position.x, msg->pose.pose.position.y, 0.001);
    has_gps = true;
}

bool setGpsState(xbot_positioning::GPSControlSrvRequest &req, xbot_positioning::GPSControlSrvResponse &res) {
    //gps_enabled = req.gps_enabled;
    return true;
}

bool setPose(xbot_positioning::SetPoseSrvRequest &req, xbot_positioning::SetPoseSrvResponse &res) {
    tf2::Quaternion q;
    tf2::fromMsg(req.robot_pose.orientation, q);


    tf2::Matrix3x3 m(q);
    double unused1, unused2, yaw;

    m.getRPY(unused1, unused2, yaw);
    core.setState(req.robot_pose.position.x, req.robot_pose.position.y, yaw,0,0);
    return true;
}

int main(int argc, char** argv) {
    // Initialize laser projector
    ros::init(argc, argv, "om_slam_node");

    ros::NodeHandle n;

    ros::ServiceServer gps_service = n.advertiseService("/xbot_positioning/set_gps_state", setGpsState);
    ros::ServiceServer pose_service = n.advertiseService("/xbot_positioning/set_robot_pose", setPose);
    
    // Set up subscribers and publishers
    odometry_pub = n.advertise<nav_msgs::Odometry>("/xbot_positioning/odom_out", 50);
    absolute_pose_pub_ = n.advertise<xbot_msgs::AbsolutePose>("/xbot_positioning/xb_pose", 50);
    pc_pub_ = n.advertise<sensor_msgs::PointCloud2>("/points2", 10);

    ros::Subscriber gps_pose_sub_ = n.subscribe("/xbot_driver_gps/xb_pose", 10, onGpsPose);
    ros::Subscriber imu_sub = n.subscribe("/imu/data_raw", 10, onImu);
    ros::Subscriber wheel_tick_sub = n.subscribe("/mower/wheel_ticks", 10, onWheelTicks);

    ROS_INFO("Waiting for /scan topic to be published...");
    // Wait for a single message on the topic
    boost::shared_ptr<const sensor_msgs::LaserScan> scanMsg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan", n);
    if (scanMsg) {
        ROS_INFO("Message received from /scan, proceeding to subscribe");
    } else {
        ROS_ERROR("Failed to receive a message from /scan");
        return 1;
    }
    laser_projector_ = new laser_geometry::LaserProjection();

    ros::Subscriber scan_sub_ = n.subscribe("/scan", 10, scanCallback);
    ROS_INFO("om_slam_node initialized.");

    ros::AsyncSpinner spinner(0); // Use 2 threads for spinning
    spinner.start();
    ROS_INFO("Waiting for /tracked_pose topic to be published...");
    boost::shared_ptr<const geometry_msgs::PoseStamped> poseMsg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/tracked_pose", n);
    if (poseMsg) {
        ROS_INFO("Message received from /tracked_pose, proceeding to subscribe");
        ros::Subscriber pose_sub_ = n.subscribe("/tracked_pose", 50, poseCallback);
    } else {
        ROS_ERROR("Failed to receive a message from /tracked_pose");
    }
    ros::waitForShutdown();
    spinner.stop();
    ROS_INFO("om_slam_node uninitialized.");

    // Cleanup
    delete laser_projector_;

    return 0;
}