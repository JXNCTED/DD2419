#include "geometry_msgs/msg/pose_stamped.hpp"  //path
#include "geometry_msgs/msg/vector3.hpp"       //wheels
#include "nav_msgs/msg/odometry.hpp"           //odometry
#include "nav_msgs/msg/path.hpp"               //path
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"                        //general
#include "sensor_msgs/msg/imu.hpp"                  //imu
#include "tf2/LinearMath/Quaternion.h"              //odometry, path and tf
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  //tf
#include "tf2_ros/transform_broadcaster.h"          //tf

using namespace std::chrono_literals;
class FilterOdom : public rclcpp::Node
{
   public:
    FilterOdom() : Node("filter_odom")
    {
        rclcpp::SensorDataQoS sensorQos;
        // Subscriptions
        wheel_odom_sub_ =
            this->create_subscription<geometry_msgs::msg::Vector3>(
                "/wheel_odom",
                10,
                std::bind(
                    &FilterOdom::wheel_callback, this, std::placeholders::_1));
        realsense_gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/camera/gyro/sample",
            sensorQos,
            std::bind(
                &FilterOdom::realsense_callback, this, std::placeholders::_1));
        phidget_gyro_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            10,
            std::bind(
                &FilterOdom::phidget_callback, this, std::placeholders::_1));
        // Publishers
        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        // Timer
        timer_ = this->create_wall_timer(
            50ms, std::bind(&FilterOdom::timer_callback, this));
    }

   private:
    // This is the main method that combines wheel and IMU odometry and
    // publishes it as odom, path and tf.
    void timer_callback()
    {
        rclcpp::Time stamp = realsense_stamp_;
        // double wheels[2]    = wheel_odom;
        // double realsense[2] = realsense_odom;
        double dt      = 50.0 / 1000.0;
        double avg_lin = wheel_odom[0];
        // double avg_ang = (wheel_odom[1] + realsense_odom[1]) / 2.0;
        double avg_ang = realsense_odom[1];

        x_ += avg_lin * cos(yaw_) * dt;
        y_ += avg_lin * sin(yaw_) * dt;
        yaw_ += avg_ang * dt;
        publish_odom(stamp);
        publish_path(stamp);
        broadcast_tf(stamp);
    }

    // Helper method that creates a odom message and publishes it.
    void publish_odom(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp         = stamp;
        odom.header.frame_id      = "odom";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_);
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom_pub_->publish(odom);
    }

    // Helper method that appends to a path the pose and publishes the path.
    void publish_path(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp     = stamp;
        pose.header.frame_id  = "odom";
        pose.pose.position.x  = x_;
        pose.pose.position.y  = y_;
        pose.pose.position.z  = 0.0;
        tf2::Quaternion q     = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_);
        pose.pose.orientation = tf2::toMsg(q);
        path_.header.stamp    = stamp;
        path_.header.frame_id = "odom";
        path_.poses.push_back(pose);
        path_pub_->publish(path_);
    }

    // Helper method that creates a tf: odom -> base_link, and publishes it.
    void broadcast_tf(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp            = stamp;
        odom_tf.header.frame_id         = "odom";
        odom_tf.child_frame_id          = "base_link";
        odom_tf.transform.translation.x = x_;
        odom_tf.transform.translation.y = y_;
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw_);
        odom_tf.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(odom_tf);
    }

    // Callbacks from wheel enconders, realsense IMU and phidget IMU.
    void wheel_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        wheel_odom[0] = msg->x;
        wheel_odom[1] = msg->y;
    }
    void realsense_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        realsense_odom[0] =
            (msg->linear_acceleration.z + _realsense_old[0]) / 2.0;
        realsense_odom[1] =
            (-msg->angular_velocity.y + _realsense_old[1]) / 2.0;  // rad/s
        if (std::abs(realsense_odom[1]) < 0.01)
            realsense_odom[1] = 0.0;
        _realsense_old[0] = msg->linear_acceleration.z;
        _realsense_old[1] = -msg->angular_velocity.y;
        realsense_stamp_  = msg->header.stamp;
    }
    void phidget_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        phidget_odom[0] =
            msg->linear_acceleration
                .x;  // ---------------- I dont know if this is the correct axis
        phidget_odom[1] = msg->angular_velocity.z;
    }

    // Pubs and subs
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr realsense_gyro_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr phidget_gyro_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr
        wheel_odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::Path path_;
    // Fields to store the subscriptions values.
    double wheel_odom[2];
    double _realsense_old[2];
    double realsense_odom[2];
    double phidget_odom[2];

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time realsense_stamp_;

    // The pose of the robot.
    float x_   = 0.0;
    float y_   = 0.0;
    float yaw_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterOdom>());
    rclcpp::shutdown();
}
