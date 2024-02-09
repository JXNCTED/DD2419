#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robp_interfaces/msg/encoders.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

class WheelOdom : public rclcpp::Node
{
   public:
    WheelOdom() : Node("wheel_odom")
    {
        path_pub_    = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
        encoder_sub_ = this->create_subscription<robp_interfaces::msg::Encoders>(
            "/motor/encoders", 10, std::bind(&WheelOdom::encodersCallback, this, std::placeholders::_1));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

   private:
    void publish_path(const rclcpp::Time &stamp, const float &x, const float &y, const float &yaw)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp     = stamp;
        pose.header.frame_id  = "odom";
        pose.pose.position.x  = x;
        pose.pose.position.y  = y;
        pose.pose.position.z  = 0.0;
        tf2::Quaternion q     = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw);
        pose.pose.orientation = tf2::toMsg(q);
        path_.header.stamp    = stamp;
        path_.header.frame_id = "odom";
        path_.poses.push_back(pose);
        path_pub_->publish(path_);
        }
    void broadcast_tf(const rclcpp::Time &stamp, const float &x, const float &y, const float &yaw)
    {
        geometry_msgs::msg::TransformStamped odom_tf;
        odom_tf.header.stamp            = stamp;
        odom_tf.header.frame_id         = "odom";
        odom_tf.child_frame_id          = "base_link";
        odom_tf.transform.translation.x = x;
        odom_tf.transform.translation.y = y;
        odom_tf.transform.translation.z = 0.0;
        tf2::Quaternion q               = tf2::Quaternion(tf2::Vector3(0, 0, 1), yaw);
        odom_tf.transform.rotation      = tf2::toMsg(q);
        tf_broadcaster_->sendTransform(odom_tf);
    }
    void encodersCallback(const robp_interfaces::msg::Encoders::SharedPtr msg)
    {
        const float TICK_PER_REV = 3600;
        const float WHEEL_RADIUS = 0.04921;
        const float WHEEL_BASE   = 0.3;

        const float DT = 50.0 / 1000.0;

        float vw1 = msg->delta_encoder_left * 2 * M_PI / TICK_PER_REV / DT;
        float vw2 = msg->delta_encoder_right * 2 * M_PI / TICK_PER_REV / DT;

        float v = (vw1 + vw2) * WHEEL_RADIUS / 2;
        float w = (vw2 - vw1) * WHEEL_RADIUS / WHEEL_BASE;

        x_ += v * DT * cos(yaw_);
        y_ += v * DT * sin(yaw_);
        yaw_ += w * DT;

        publish_path(msg->header.stamp, x_, y_, yaw_);
        broadcast_tf(msg->header.stamp, x_, y_, yaw_);
    }
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Subscription<robp_interfaces::msg::Encoders>::SharedPtr encoder_sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path path_;

    float x_   = 0.0;
    float y_   = 0.0;
    float yaw_ = 0.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdom>());
    rclcpp::shutdown();
    return 0;
}
