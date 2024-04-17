#include <chrono>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry/keyframe.hpp"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
class LidarLandmarker : public rclcpp::Node
{
   public:
    LidarLandmarker() : Node("lidar_landmarker")
    {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/valid_scan",
            10,
            std::bind(
                &LidarLandmarker::lidarCallback, this, std::placeholders::_1));

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            3s, std::bind(&LidarLandmarker::timerCallback, this));

        tf.header.frame_id         = "map";
        tf.header.stamp            = rclcpp::Time(0);
        tf.child_frame_id          = "odom";
        tf.transform.translation.x = 0;
        tf.transform.translation.y = 0;
        tf.transform.translation.z = 0;
        tf.transform.rotation.w    = 1;
        tf.transform.rotation.x    = 0;
        tf.transform.rotation.y    = 0;
        tf.transform.rotation.z    = 0;

        tf_broadcaster_->sendTransform(tf);
    }

   private:
    void timerCallback()
    {
        if (keyframes.empty())
        {
            return;
        }
        const auto &keyframeCloud = keyframes.back().getCloud();
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(lastCloudMsg, *cloud);

        icp.setInputSource(cloud);
        icp.setInputTarget(keyframeCloud.makeShared());
        icp.setMaximumIterations(100);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);
        RCLCPP_INFO(this->get_logger(),
                    "ICP converged: %d, fitness score: %f",
                    icp.hasConverged(),
                    icp.getFitnessScore());
        if (not icp.hasConverged() or icp.getFitnessScore() > 2.0)
        {
            return;
        }

        Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
        odom.block<3, 3>(0, 0) =
            Eigen::Quaterniond(tf_odom.transform.rotation.w,
                               tf_odom.transform.rotation.x,
                               tf_odom.transform.rotation.y,
                               tf_odom.transform.rotation.z)
                .toRotationMatrix();
        odom.block<3, 1>(0, 3) << tf_odom.transform.translation.x,
            tf_odom.transform.translation.y, tf_odom.transform.translation.z;

        Eigen::Matrix4d odomInv   = Eigen::Matrix4d::Identity();
        odomInv.block<3, 3>(0, 0) = odom.block<3, 3>(0, 0).transpose();
        odomInv.block<3, 1>(0, 3) =
            -odom.block<3, 3>(0, 0).transpose() * odom.block<3, 1>(0, 3);

        Eigen::Matrix4d icpTF = icp.getFinalTransformation().cast<double>();

        Eigen::Matrix4d diff = odomInv * icpTF;

        tf.transform.translation.x = diff(0, 3);
        tf.transform.translation.y = diff(1, 3);
        tf.transform.translation.z = diff(2, 3);
        tf.transform.rotation.w    = 1;
        tf.transform.rotation.x    = 0;
        tf.transform.rotation.y    = 0;
        tf.transform.rotation.z    = 0;
    }
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        static bool first = false;
        if (not first)
        {
            first = true;
            return;
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        lastCloudMsg = *msg;

        if (keyframes.empty())
        {
            // homogeneous transformation of identity transform
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            // keyframes.push_back(Keyframe<pcl::PointXYZ>(cloud, pose));
            keyframes.push_back(Keyframe<pcl::PointXYZ>(*cloud, pose));
            RCLCPP_INFO(this->get_logger(), "Added first keyframe");
        }

        try
        {
            tf_odom = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        tf.header.stamp = msg->header.stamp;
        tf_broadcaster_->sendTransform(tf);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped tf_odom;
    geometry_msgs::msg::TransformStamped tf;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    sensor_msgs::msg::PointCloud2 lastCloudMsg;
    std::vector<Keyframe<pcl::PointXYZ>> keyframes;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarLandmarker>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}