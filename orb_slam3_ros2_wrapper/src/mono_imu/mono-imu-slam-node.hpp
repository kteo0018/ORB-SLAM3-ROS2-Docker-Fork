/**
 * @file mono-imu-slam-node.hpp
 * @brief Definition of the MonoImuSlamNode Wrapper class (pure monocular + IMU).
 */

#ifndef MONO_IMU_SLAM_NODE_HPP_
#define MONO_IMU_SLAM_NODE_HPP_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "std_srvs/srv/set_bool.hpp"

#include <slam_msgs/msg/map_data.hpp>
#include <slam_msgs/msg/slam_info.hpp>
#include <slam_msgs/srv/get_map.hpp>

#include "orb_slam3_ros2_wrapper/type_conversion.hpp"
#include "orb_slam3_ros2_wrapper/orb_slam3_interface.hpp"

namespace ORB_SLAM3_Wrapper
{
    class MonoImuSlamNode : public rclcpp::Node
    {
    public:
        MonoImuSlamNode(const std::string &strVocFile,
                        const std::string &strSettingsFile,
                        ORB_SLAM3::System::eSensor sensor);
        ~MonoImuSlamNode();

    private:
        // ROS 2 Callbacks.
        void ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU);
        void ImageCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB);

        void publishMapData();

        void getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                          std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                          std::shared_ptr<slam_msgs::srv::GetMap::Response> response);

        // Subscriptions
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgbSub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imuSub_;

        // Publishers
        rclcpp::Publisher<slam_msgs::msg::MapData>::SharedPtr mapDataPub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr robotPoseMapFrame_;
        rclcpp::Publisher<slam_msgs::msg::SlamInfo>::SharedPtr slamInfoPub_;

        // TF
        std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
        std::shared_ptr<tf2_ros::TransformListener> tfListener_;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer_;

        // Services
        rclcpp::Service<slam_msgs::srv::GetMap>::SharedPtr getMapDataService_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr resetLocalMapSrv_;

        // Timers
        rclcpp::TimerBase::SharedPtr mapDataTimer_;
        rclcpp::CallbackGroup::SharedPtr mapDataCallbackGroup_;

        // ROS Params
        std::string robot_base_frame_id_;
        std::string odom_frame_id_;
        std::string global_frame_;
        double robot_x_, robot_y_, robot_z_, robot_qx_, robot_qy_, robot_qz_, robot_qw_;
        bool isTracked_ = false;
        bool odometry_mode_;
        bool publish_tf_;
        double frequency_tracker_count_ = 0;
        int map_data_publish_frequency_;
        bool do_loop_closing_;
        std::chrono::_V2::system_clock::time_point frequency_tracker_clock_;

        std::shared_ptr<ORB_SLAM3_Wrapper::ORBSLAM3Interface> interface_;
        geometry_msgs::msg::TransformStamped tfMapOdom_;
    };
}

#endif


