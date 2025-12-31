// Copyright (c) 2025, Hammad Ur Rehman
// All rights reserved.

#include <memory>
#include <string>
#include <thread>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

namespace sim_bridge
{

class SimBridgeNode : public rclcpp::Node
{
public:
    SimBridgeNode() : Node("sim_bridge_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing Isaac Sim Bridge Node");

        // Initialize parameters
        this->declare_parameter<std::string>("isaac_sim_ip", "localhost");
        this->declare_parameter<int>("isaac_sim_port", 50051);
        this->declare_parameter<bool>("ros_bridge_enabled", true);
        this->declare_parameter<double>("physics_update_rate", 60.0);
        this->declare_parameter<double>("rendering_update_rate", 30.0);
        this->declare_parameter<double>("fixed_timestep", 0.001);

        // Get parameters
        isaac_sim_ip_ = this->get_parameter("isaac_sim_ip").as_string();
        isaac_sim_port_ = this->get_parameter("isaac_sim_port").as_int();
        ros_bridge_enabled_ = this->get_parameter("ros_bridge_enabled").as_bool();
        physics_update_rate_ = this->get_parameter("physics_update_rate").as_double();
        rendering_update_rate_ = this->get_parameter("rendering_update_rate").as_double();
        fixed_timestep_ = this->get_parameter("fixed_timestep").as_double();

        // Initialize publishers for sensor data
        camera_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/sim/camera/image_raw", 10);
        lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/sim/lidar/scan", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/sim/imu/data", 10);

        // Initialize TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // Initialize timers for sim updates
        physics_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / physics_update_rate_)),
            std::bind(&SimBridgeNode::physics_update_callback, this));

        rendering_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / rendering_update_rate_)),
            std::bind(&SimBridgeNode::rendering_update_callback, this));

        RCLCPP_INFO(this->get_logger(), "Isaac Sim Bridge Node initialized successfully");
    }

private:
    // Parameters
    std::string isaac_sim_ip_;
    int isaac_sim_port_;
    bool ros_bridge_enabled_;
    double physics_update_rate_;
    double rendering_update_rate_;
    double fixed_timestep_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr camera_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;

    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timers
    rclcpp::TimerBase::SharedPtr physics_timer_;
    rclcpp::TimerBase::SharedPtr rendering_timer_;

    // Callback functions
    void physics_update_callback()
    {
        if (!ros_bridge_enabled_) return;

        // Simulate physics update - in real implementation, this would interface with Isaac Sim
        RCLCPP_DEBUG(this->get_logger(), "Physics update at %.2f Hz", physics_update_rate_);

        // Publish TF transforms
        publish_transforms();
    }

    void rendering_update_callback()
    {
        if (!ros_bridge_enabled_) return;

        // Simulate rendering update - in real implementation, this would interface with Isaac Sim
        RCLCPP_DEBUG(this->get_logger(), "Rendering update at %.2f Hz", rendering_update_rate_);

        // Publish sensor data
        publish_camera_data();
        publish_lidar_data();
        publish_imu_data();
    }

    void publish_transforms()
    {
        // Create and publish a transform (example: robot base to camera)
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "robot_base";
        t.child_frame_id = "camera_frame";

        t.transform.translation.x = 0.1;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.2;
        t.transform.rotation.x = 0.0;
        t.transform.rotation.y = 0.0;
        t.transform.rotation.z = 0.0;
        t.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(t);
    }

    void publish_camera_data()
    {
        // Create a simulated image
        cv::Mat image = cv::Mat(480, 640, CV_8UC3, cv::Scalar(128, 128, 128)); // Gray image for simulation

        // Add some simulated objects
        cv::circle(image, cv::Point(320, 240), 50, cv::Scalar(0, 255, 0), -1); // Green circle

        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        msg->header.stamp = this->get_clock()->now();
        msg->header.frame_id = "camera_frame";

        camera_publisher_->publish(*msg);
    }

    void publish_lidar_data()
    {
        auto msg = sensor_msgs::msg::LaserScan();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "lidar_frame";

        msg.angle_min = -M_PI / 2;
        msg.angle_max = M_PI / 2;
        msg.angle_increment = M_PI / 180.0; // 1 degree
        msg.time_increment = 0.0;
        msg.scan_time = 1.0 / rendering_update_rate_;
        msg.range_min = 0.1;
        msg.range_max = 25.0;

        int num_readings = static_cast<int>((msg.angle_max - msg.angle_min) / msg.angle_increment);
        msg.ranges.resize(num_readings);
        msg.intensities.resize(num_readings);

        // Simulate some readings (in a real implementation, this would come from Isaac Sim)
        for (int i = 0; i < num_readings; ++i) {
            msg.ranges[i] = 10.0; // Simulate distance of 10m
            msg.intensities[i] = 100.0;
        }

        lidar_publisher_->publish(msg);
    }

    void publish_imu_data()
    {
        auto msg = sensor_msgs::msg::Imu();
        msg.header.stamp = this->get_clock()->now();
        msg.header.frame_id = "imu_frame";

        // Simulate IMU data (in a real implementation, this would come from Isaac Sim)
        msg.orientation.x = 0.0;
        msg.orientation.y = 0.0;
        msg.orientation.z = 0.0;
        msg.orientation.w = 1.0;

        msg.angular_velocity.x = 0.01;
        msg.angular_velocity.y = 0.01;
        msg.angular_velocity.z = 0.01;

        msg.linear_acceleration.x = 0.1;
        msg.linear_acceleration.y = 0.1;
        msg.linear_acceleration.z = 9.81; // Gravity

        imu_publisher_->publish(msg);
    }
};

} // namespace sim_bridge

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<sim_bridge::SimBridgeNode>());
    rclcpp::shutdown();
    return 0;
}