#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"

class FilterOdom : public rclcpp::Node
{
   public:
    FilterOdom() : node("filter_odom")
    {
        wheel_odom_sub_ =
            this->create_subscription<geometry_msgs::msg::Twist>("", 10, std::bind(&FilterOdom::wheel_callback, this, std::placeholders::_1));
        realsense_gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/gyro/sample", 10, std::bind(&FilterOdom::realsense_callback, this, std::placeholders::_1));
        phidget_gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw", 10, std::bind(&FilterOdom::phidget_callback, this, std::placeholders::_1));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_    = this->create_wall_timer(50ms, std::bind(&FilterOdom::timer_callback, this));
    }

    void timer_callback()
    {
        double dt      = 50.0 / 1000.0;
        double avg_lin = (wheel_odom[0] + realsense_odom[0]) / 2.0;
        double avg_ang = (wheel_odom[1] + realsense_odom[1]) / 2.0;
        x_ += avg_lin * sin(yaw_) * dt;
        y_ += avg_lin * cos(yaw_) * dt;
        yaw_ += avg_ang * dt;
        publish_odom();
    }

   private:
    void publish_odom()
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp          = realsense_stamp_;
        odom.header.frame_id       = "odom";
        odom.pose.pose.position.x  = x_;
        odom.pose.pose.position.y  = y_;
        odom.pose.pose.position.z  = 0.0;
        tf2::Quaternion q          = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom_pub_->publish(odom);
    }

    void wheel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        wheel_odom[0] = msg->linear.x;
        wheel_odom[1] = msg->angular.z;
    }
    void realsense_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        realsense_odom[0] = (msg->linear_acceleration.z + _realsense_old[0]) / 2.0;  // ---------------- I dont know if this is the correct axis
        realsense_odom[1] = (msg->angular_velocity.y + _realsense_old[1]) / 2.0;     // rad/s
        _realsense_old[0] = msg->linear_acceleration.z;
        _realsense_old[1] = msg->angular_velocity.y;
        realsense_stamp_  = msg->header.stamp;
    }
    void phidget_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        phidget_odom[0] = msg->linear_acceleration.x;  // ---------------- I dont know if this is the correct axis
        phidget_odom[1] = msg->angular_velocity.z;
    }
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr realsense_gyro_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr phidget_gyro_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr wheel_odom_sub_;
    double wheel_odom[2];
    double _realsense_old[2];
    double realsense_odom[2];
    double phidget_odom[2];
    rclcpp::TimerBase::SharedPtr timer_;
    float x_   = 0.0;
    float y_   = 0.0;
    float yaw_ = 0.0;
    rclcpp::Time realsense_stamp_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterOdom>());
    rclcpp::shutdown();
}