/**
 * @file mono-imu-slam-node.cpp
 * @brief Implementation of the MonoImuSlamNode Wrapper class (pure monocular + IMU).
 */

#include "mono-imu-slam-node.hpp"

namespace ORB_SLAM3_Wrapper
{
    MonoImuSlamNode::MonoImuSlamNode(const std::string &strVocFile,
                                     const std::string &strSettingsFile,
                                     ORB_SLAM3::System::eSensor sensor)
        : Node("ORB_SLAM3_MONO_IMU_ROS2")
    {
        // Declare parameters (topic names)
        // this->declare_parameter("rgb_image_topic_name", rclcpp::ParameterValue("camera/image_raw"));
        // this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("imu"));
        this->declare_parameter("rgb_image_topic_name", rclcpp::ParameterValue("/world/z_simple_palm_plantation/model/x500_custom_0/link/rgbd_link/sensor/rgbd/image"));
        this->declare_parameter("imu_topic_name", rclcpp::ParameterValue("/world/z_simple_palm_plantation/model/x500_custom_0/link/imu_link/sensor/imu/imu"));

        // ROS Subscribers
        rgbSub_ = this->create_subscription<sensor_msgs::msg::Image>(
            this->get_parameter("rgb_image_topic_name").as_string(), 10,
            std::bind(&MonoImuSlamNode::ImageCallback, this, std::placeholders::_1));

        // DEBUG 01: REMOVE LATER
        // std::string debug_rgb_name;
        // this->get_parameter("rgb_image_topic_name", debug_rgb_name);
        // RCLCPP_ERROR(this->get_logger(), "DEBUG: I am trying to subscribe to Camera Topic: %s", debug_rgb_name.c_str());

        rclcpp::QoS imu_qos(rclcpp::KeepLast(1000));
        imu_qos.best_effort(); // Essential for PX4/Gazebo IMUs
        imuSub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            this->get_parameter("imu_topic_name").as_string(), imu_qos,
            std::bind(&MonoImuSlamNode::ImuCallback, this, std::placeholders::_1));

        // ROS Publishers
        mapDataPub_ = this->create_publisher<slam_msgs::msg::MapData>("map_data", 10);
        robotPoseMapFrame_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("robot_pose_slam", 10);
        slamInfoPub_ = this->create_publisher<slam_msgs::msg::SlamInfo>("slam_info", 10);

        // Services
        getMapDataService_ = this->create_service<slam_msgs::srv::GetMap>(
            "orb_slam3/get_map_data",
            std::bind(&MonoImuSlamNode::getMapServer, this,
                      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        resetLocalMapSrv_ = this->create_service<std_srvs::srv::SetBool>(
            "orb_slam3/reset_mapping",
            [this](std::shared_ptr<rmw_request_id_t>,
                   std::shared_ptr<std_srvs::srv::SetBool::Request>,
                   std::shared_ptr<std_srvs::srv::SetBool::Response>) {
                interface_->resetLocalMapping();
            });

        // TF
        tfBroadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        tfBuffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);

        bool bUseViewer;
        this->declare_parameter("visualization", rclcpp::ParameterValue(true));
        this->get_parameter("visualization", bUseViewer);

        this->declare_parameter("robot_base_frame", "base_footprint");
        this->get_parameter("robot_base_frame", robot_base_frame_id_);

        this->declare_parameter("global_frame", "map");
        this->get_parameter("global_frame", global_frame_);

        this->declare_parameter("odom_frame", "odom");
        this->get_parameter("odom_frame", odom_frame_id_);

        this->declare_parameter("robot_x", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_x", robot_x_);

        this->declare_parameter("robot_y", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_y", robot_y_);

        this->declare_parameter("robot_z", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_z", robot_z_);

        // Declare and get the quaternion components
        this->declare_parameter("robot_qx", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qx", robot_qx_);

        this->declare_parameter("robot_qy", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qy", robot_qy_);

        this->declare_parameter("robot_qz", rclcpp::ParameterValue(0.0));
        this->get_parameter("robot_qz", robot_qz_);

        this->declare_parameter("robot_qw", rclcpp::ParameterValue(1.0));
        this->get_parameter("robot_qw", robot_qw_);

        // Create and populate the Pose message
        geometry_msgs::msg::Pose initial_pose;
        initial_pose.position.x = robot_x_;
        initial_pose.position.y = robot_y_;
        initial_pose.position.z = robot_z_;
        initial_pose.orientation.x = robot_qx_;
        initial_pose.orientation.y = robot_qy_;
        initial_pose.orientation.z = robot_qz_;
        initial_pose.orientation.w = robot_qw_;

        this->declare_parameter("odometry_mode", rclcpp::ParameterValue(false));
        this->get_parameter("odometry_mode", odometry_mode_);

        this->declare_parameter("publish_tf", rclcpp::ParameterValue(true));
        this->get_parameter("publish_tf", publish_tf_);

        this->declare_parameter("map_data_publish_frequency", rclcpp::ParameterValue(1000));
        this->get_parameter("map_data_publish_frequency", map_data_publish_frequency_);

        this->declare_parameter("do_loop_closing", rclcpp::ParameterValue(true));
        this->get_parameter("do_loop_closing", do_loop_closing_);

        // Timers
        mapDataCallbackGroup_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        mapDataTimer_ = this->create_wall_timer(
            std::chrono::milliseconds(map_data_publish_frequency_),
            std::bind(&MonoImuSlamNode::publishMapData, this),
            mapDataCallbackGroup_);

        interface_ = std::make_shared<ORB_SLAM3_Wrapper::ORBSLAM3Interface>(
            strVocFile, strSettingsFile,
            sensor, bUseViewer, do_loop_closing_, initial_pose,
            global_frame_, odom_frame_id_, robot_base_frame_id_);

        frequency_tracker_count_ = 0;
        frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();

        RCLCPP_INFO(this->get_logger(), "MonoImuSlamNode CONSTRUCTOR END!");
    }

    MonoImuSlamNode::~MonoImuSlamNode()
    {
        rgbSub_.reset();
        imuSub_.reset();
        interface_.reset();
        RCLCPP_INFO(this->get_logger(), "MonoImuSlamNode DESTRUCTOR!");
    }

    void MonoImuSlamNode::ImuCallback(const sensor_msgs::msg::Imu::SharedPtr msgIMU)
    {
        // DEBUG 03: REMOVE LATER
        // Only print once every 50 times to avoid spamming console
        static int debug_counter = 0;
        if (debug_counter++ % 50 == 0) {
            RCLCPP_ERROR(this->get_logger(), "DEBUG: IMU Callback Triggered!"); // <--- ADD THIS
        }
        RCLCPP_DEBUG_STREAM(this->get_logger(), "ImuCallback");
        interface_->handleIMU(msgIMU);
    }

    void MonoImuSlamNode::ImageCallback(const sensor_msgs::msg::Image::SharedPtr msgRGB)
    {
        // DEBUG 02: REMOVE LATER
        // RCLCPP_ERROR(this->get_logger(), "DEBUG: Image Callback Triggered!"); // <--- ADD THIS
        Sophus::SE3f Tcw;
        if (interface_->trackMonocularIMU(msgRGB, Tcw))
        {
            isTracked_ = true;

            if (publish_tf_)
            {
                if (!odometry_mode_)
                {
                    auto tfMapRobot = geometry_msgs::msg::TransformStamped();
                    tfMapRobot.header.stamp = msgRGB->header.stamp;
                    tfMapRobot.header.frame_id = global_frame_;
                    tfMapRobot.child_frame_id = odom_frame_id_;
                    interface_->getDirectMapToRobotTF(msgRGB->header, tfMapRobot);
                    tfBroadcaster_->sendTransform(tfMapRobot);
                }
                else
                {
                    try
                    {
                        auto msgOdom = tfBuffer_->lookupTransform(odom_frame_id_, robot_base_frame_id_, tf2::TimePointZero);
                        interface_->getMapToOdomTF(msgOdom, tfMapOdom_);
                        tfBroadcaster_->sendTransform(tfMapOdom_);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                                    odom_frame_id_.c_str(), robot_base_frame_id_.c_str(), ex.what());
                        RCLCPP_ERROR(this->get_logger(), "You have set the parameter `odometry_mode` to true. This requires the transform between `odom_frame` and `robot_base_frame` to exist. If you do not have odometry information, set odometry_mode to false.");
                        return;
                    }
                }
            }

            geometry_msgs::msg::PoseStamped pose;
            interface_->getRobotPose(pose);
            pose.header.stamp = msgRGB->header.stamp;
            robotPoseMapFrame_->publish(pose);

            ++frequency_tracker_count_;
        }
    }

    void MonoImuSlamNode::publishMapData()
    {
        if (isTracked_)
        {
            auto start = std::chrono::high_resolution_clock::now();
            slam_msgs::msg::SlamInfo slamInfoMsg;
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Publishing map data");
            double tracking_freq = frequency_tracker_count_ /
                                   std::chrono::duration_cast<std::chrono::duration<double>>(
                                       std::chrono::high_resolution_clock::now() - frequency_tracker_clock_)
                                       .count();
            RCLCPP_INFO_STREAM(this->get_logger(), "Current ORB-SLAM3 tracking frequency: " << tracking_freq << " frames / sec");
            frequency_tracker_clock_ = std::chrono::high_resolution_clock::now();
            frequency_tracker_count_ = 0;

            slam_msgs::msg::MapData mapDataMsg;
            interface_->mapDataToMsg(mapDataMsg, true, false);
            mapDataPub_->publish(mapDataMsg);
            slamInfoMsg.num_maps = interface_->getNumberOfMaps();
            slamInfoMsg.num_keyframes_in_current_map = mapDataMsg.graph.poses_id.size();
            slamInfoMsg.tracking_frequency = tracking_freq;
            auto t1 = std::chrono::high_resolution_clock::now();
            auto time_publishMapData = std::chrono::duration_cast<std::chrono::duration<double>>(t1 - start).count();
            RCLCPP_DEBUG_STREAM(this->get_logger(), "Time to create mapdata: " << time_publishMapData << " seconds");
            RCLCPP_INFO_STREAM(this->get_logger(), "*************************");
            slamInfoPub_->publish(slamInfoMsg);
        }
    }

    void MonoImuSlamNode::getMapServer(std::shared_ptr<rmw_request_id_t> request_header,
                                       std::shared_ptr<slam_msgs::srv::GetMap::Request> request,
                                       std::shared_ptr<slam_msgs::srv::GetMap::Response> response)
    {
        (void)request_header;
        RCLCPP_INFO(this->get_logger(), "GetMap service called (mono_imu).");
        slam_msgs::msg::MapData mapDataMsg;
        interface_->mapDataToMsg(mapDataMsg, false, request->tracked_points, request->kf_id_for_landmarks);
        response->data = mapDataMsg;
    }
}


