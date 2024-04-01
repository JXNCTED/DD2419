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
        // place holder, publish zero tf
        // geometry_msgs::msg::TransformStamped tf;
        // tf.header.stamp            = msg->header.stamp;
        // tf.header.frame_id         = "map";
        // tf.child_frame_id          = "odom";
        // tf.transform.translation.x = 0;
        // tf.transform.translation.y = 0;
        // tf.transform.translation.z = 0;
        // tf.transform.rotation.x    = 0;
        // tf.transform.rotation.y    = 0;
        // tf.transform.rotation.z    = 0;
        // tf.transform.rotation.w    = 1;
        // tf_broadcaster_->sendTransform(tf);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);

        if (keyframes.empty())
        {
            Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
            keyframes.push_back(Keyframe<pcl::PointXYZ>(cloud, pose));
            return;
        }

        // compare the current cloud with the last keyframe
        // if the difference is large enough, add the current cloud as a new
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud);
        icp.setInputTarget(keyframes.back().getCloud());
        icp.setMaximumIterations(100);
        pcl::PointCloud<pcl::PointXYZ> aligned;
        icp.align(aligned);

        if (icp.hasConverged())
        {
            Eigen::Matrix4d poseAtKeyframe =
                icp.getFinalTransformation().cast<double>();
            // get odom to map transform
            Eigen::Matrix4d odomToMap = Eigen::Matrix4d::Identity();
            for (auto &keyframe : keyframes)
            {
                odomToMap = keyframe.getPose() * odomToMap;
            }
            odomToMap = odomToMap.inverse();
            odomToMap = odomToMap * poseAtKeyframe;

            geometry_msgs::msg::TransformStamped tf;
            tf.header.stamp            = msg->header.stamp;
            tf.header.frame_id         = "map";
            tf.child_frame_id          = "odom";
            tf.transform.translation.x = odomToMap(0, 3);
            tf.transform.translation.y = odomToMap(1, 3);
            tf.transform.translation.z = odomToMap(2, 3);
            Eigen::Quaterniond q(odomToMap.block<3, 3>(0, 0));
            tf.transform.rotation.x = q.x();
            tf.transform.rotation.y = q.y();
            tf.transform.rotation.z = q.z();
            tf.transform.rotation.w = q.w();
            tf_broadcaster_->sendTransform(tf);

            if (icp.getFitnessScore() > 0.1)
            {
                Eigen::Matrix4d pose =
                    keyframes.back().getPose() * poseAtKeyframe;
                RCLCPP_INFO(this->get_logger(),
                            "New keyframe added with fitness %f",
                            icp.getFitnessScore());
                keyframes.push_back(Keyframe<pcl::PointXYZ>(cloud, pose));
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