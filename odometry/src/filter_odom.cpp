#include "geometry_msgs/msg/pose_stamped.hpp"     //path
#include "geometry_msgs/msg/vector3_stamped.hpp"  //wheel_odom
#include "nav_msgs/msg/odometry.hpp"              //odometry
#include "nav_msgs/msg/path.hpp"                  //path
#include "odometry/ekf.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"                        //general
#include "sensor_msgs/msg/imu.hpp"                  //imu
#include "tf2/LinearMath/Quaternion.h"              //odometry, path and tf
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  //tf
#include "tf2_ros/transform_broadcaster.h"          //tf

using namespace std::chrono_literals;

EKF::ProcessModelFunc processModel = [](Eigen::VectorXd x,
                                        Eigen::VectorXd u,
                                        const double &dt) -> Eigen::VectorXd
{
    Eigen::VectorXd x_next = Eigen::VectorXd::Zero(x.size());
    return x_next;
};

EKF::ObservationModelFunc observationModel = [](Eigen::VectorXd x)
{
    Eigen::VectorXd z = Eigen::VectorXd::Zero(3);
    return z;
};

EKF::VecMatFunc jacobianF = [](Eigen::VectorXd x) -> Eigen::MatrixXd
{
    Eigen::MatrixXd F = Eigen::MatrixXd::Zero(x.size(), x.size());
    return F;
};

class FilterOdom : public rclcpp::Node
{
   public:
    FilterOdom()
        : Node("filter_odom"), ekf(5, processModel, observationModel, jacobianF)
    {
        rclcpp::SensorDataQoS sensorQos;
        // Subscriptions
        wheel_odom_sub_ =
            this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
                "/wheel_odom",
                10,
                std::bind(
                    &FilterOdom::wheel_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            sensorQos,
            std::bind(
                &FilterOdom::realsense_callback, this, std::placeholders::_1));
        // Publishers
        odom_pub_ =
            this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

   private:
    // Helper method that creates a odom message and publishes it.
    void publish_odom(const rclcpp::Time &stamp)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp         = stamp;
        odom.header.frame_id      = "odom";
        odom.pose.pose.position.x = ekf.getState()(0);
        odom.pose.pose.position.y = ekf.getState()(1);
        odom.pose.pose.position.z = 0.0;
        tf2::Quaternion q =
            tf2::Quaternion(tf2::Vector3(0, 0, 1), ekf.getState()(2));
        odom.pose.pose.orientation = tf2::toMsg(q);

        odom_pub_->publish(odom);
    }

    // Helper method that appends to a path the pose and publishes the path.
    void publish_path(const rclcpp::Time &stamp)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp    = stamp;
        pose.header.frame_id = "odom";
        pose.pose.position.x = ekf.getState()(0);
        pose.pose.position.y = ekf.getState()(1);
        pose.pose.position.z = 0.0;
        tf2::Quaternion q =
            tf2::Quaternion(tf2::Vector3(0, 0, 1), ekf.getState()(2));
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
        odom_tf.transform.translation.x = ekf.getState()(0);
        odom_tf.transform.translation.y = ekf.getState()(1);
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q =
            tf2::Quaternion(tf2::Vector3(0, 0, 1), ekf.getState()(2));
        odom_tf.transform.rotation = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(odom_tf);
    }

    // Callbacks from wheel enconders, realsense IMU and phidget IMU.
    void wheel_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        // static double last_time = rclcpp::Time(msg->header.stamp).seconds();
        // double dt = rclcpp::Time(msg->header.stamp).seconds() - last_time;
        // last_time = rclcpp::Time(msg->header.stamp).seconds();
        // if (dt <= 0.0)
        //     return;
        // double v = msg->vector.x;
        // x_ += v * cos(yaw_) * dt;
        // y_ += v * sin(yaw_) * dt;
    }
    void realsense_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // static double last_time = rclcpp::Time(msg->header.stamp).seconds();
        // double dt = rclcpp::Time(msg->header.stamp).seconds() - last_time;
        // last_time = rclcpp::Time(msg->header.stamp).seconds();
        // if (dt <= 0.0)
        //     return;
        // double yaw_rate = 0.0;
        // if (std::abs(msg->angular_velocity.z) < 0.01)
        //     yaw_rate = 0.0;
        // else
        //     yaw_rate = -msg->angular_velocity.z;

        // yaw_ = yaw_ + yaw_rate * dt;

        // publish_odom(msg->header.stamp);
        // publish_path(msg->header.stamp);
        // broadcast_tf(msg->header.stamp);
    }

    // Pubs and subs
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
        wheel_odom_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

    nav_msgs::msg::Path path_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time realsense_stamp_;

    EKF ekf;
};

auto main(int argc, char **argv) -> int
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterOdom>());
    rclcpp::shutdown();
}
