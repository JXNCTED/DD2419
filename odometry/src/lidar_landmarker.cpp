#include <rclcpp/logging.hpp>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry/keyframe.hpp"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/transform_broadcaster.h"

class LidarLandmarker : public rclcpp::Node
{
   public:
    LidarLandmarker() : Node("lidar_landmarker")
    {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/compensated_scan_pc",
            10,
            std::bind(
                &LidarLandmarker::lidarCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(
                &LidarLandmarker::odomCallback, this, std::placeholders::_1));
        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

   private:
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        // geometry_msgs::msg::TransformStamped tf;
        // tf.header          = msg->header;
        // tf.child_frame_id  = "map";
        // tf.header.frame_id = "odom";

        // tf.transform.translation.x = 1;
        // tf.transform.translation.y = 0;
        // tf.transform.translation.z = 0;
        // Eigen::Quaterniond q(Eigen::Matrix3d::Identity());
        // tf.transform.rotation.w = q.w();
        // tf.transform.rotation.x = q.x();
        // tf.transform.rotation.y = q.y();
        // tf.transform.rotation.z = q.z();
        // tf_broadcaster_->sendTransform(tf);
        // return;

        if (keyframes.empty())
        {
            // homogeneous transformation of identity transform
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            // keyframes.push_back(Keyframe<pcl::PointXYZ>(cloud, pose));
            keyframes.push_back(Keyframe<pcl::PointXYZ>(*cloud, pose));
            RCLCPP_INFO(this->get_logger(), "Added first keyframe");
            return;
        }

        for (auto keyframe = keyframes.begin(); keyframe != keyframes.end();
             ++keyframe)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(cloud);
            icp.setInputTarget(keyframe->getCloud().makeShared());
            icp.setMaximumIterations(15);
            pcl::PointCloud<pcl::PointXYZ> aligned;
            icp.align(aligned);

            if (not icp.hasConverged())
            {
                continue;
            }

            if (icp.getFitnessScore() < 0.1)
            {
                // find the difference between odom and the transform
                Eigen::Matrix4d odom = Eigen::Matrix4d::Identity();
                odom.block<3, 3>(0, 0) =
                    Eigen::Quaterniond(tf_odom.transform.rotation.w,
                                       tf_odom.transform.rotation.x,
                                       tf_odom.transform.rotation.y,
                                       tf_odom.transform.rotation.z)
                        .toRotationMatrix();
                odom.block<3, 1>(0, 3) << tf_odom.transform.translation.x,
                    tf_odom.transform.translation.y,
                    tf_odom.transform.translation.z;

                // Eigen::Matrix4d icpTF =
                //     keyframe->getPose() *
                //     icp.getFinalTransformation().cast<double>();

                Eigen::Matrix4d icpTF =
                    icp.getFinalTransformation().cast<double>();
                Eigen::Matrix4d diff = odom.inverse() * icpTF;

                // publish the transform
                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp            = msg->header.stamp;
                tf.header.frame_id         = "map";
                tf.child_frame_id          = "odom";
                tf.transform.translation.x = diff(0, 3);
                tf.transform.translation.y = diff(1, 3);
                tf.transform.translation.z = diff(2, 3);
                Eigen::Quaterniond q(diff.block<3, 3>(0, 0));
                tf.transform.rotation.w = q.w();
                tf.transform.rotation.x = q.x();
                tf.transform.rotation.y = q.y();
                tf.transform.rotation.z = q.z();
                tf_broadcaster_->sendTransform(tf);
                RCLCPP_INFO(this->get_logger(),
                            "Published transform: %f %f %f",
                            tf.transform.translation.x,
                            tf.transform.translation.y,
                            tf.transform.translation.z);
                break;
            }
            else if (std::next(keyframe) == keyframes.end())
            {
                // Check if keyframe is a valid iterator
                // homogeneous transformation of identity transform
                Eigen::Matrix4d pose =
                    keyframe->getPose() *
                    icp.getFinalTransformation().cast<double>();

                // Check if pose is a valid matrix
                keyframes.push_back(Keyframe<pcl::PointXYZ>(*cloud, pose));
                RCLCPP_INFO(this->get_logger(), "Added new keyframe");
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf_odom.header.stamp            = msg->header.stamp;
        tf_odom.header.frame_id         = "odom";
        tf_odom.child_frame_id          = "base_link";
        tf_odom.transform.translation.x = msg->pose.pose.position.x;
        tf_odom.transform.translation.y = msg->pose.pose.position.y;
        tf_odom.transform.translation.z = msg->pose.pose.position.z;
        tf_odom.transform.rotation      = msg->pose.pose.orientation;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    geometry_msgs::msg::TransformStamped tf_odom;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

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