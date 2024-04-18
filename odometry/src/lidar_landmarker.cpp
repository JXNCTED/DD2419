#include <chrono>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "odometry/keyframe.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/registration/icp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
// #include "tf2_eigen/tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;
class LidarLandmarker : public rclcpp::Node
{
   public:
    LidarLandmarker() : Node("lidar_landmarker")
    {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/valid_scan",
            10,
            std::bind(
                &LidarLandmarker::lidarCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(
                &LidarLandmarker::odomCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/map_pc", 10);

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ =
            std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

        timer_ = this->create_wall_timer(
            1s, std::bind(&LidarLandmarker::timerCallback, this));

        T_map_odom.setIdentity();
    }

   private:
    void timerCallback()
    {
        if (lastCloud.empty() or mapCloud.empty())
        {
            return;
        }
        const static double ICP_THRESHOLD = 0.01;
        // inital guess
        Eigen::Matrix4d T_map_base_guess = T_map_odom * T_odom_base;

        // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(lastCloud.makeShared());
        icp.setInputTarget(mapCloud.makeShared());
        icp.setRANSACOutlierRejectionThreshold(0.1);
        pcl::PointCloud<pcl::PointXYZ> final;
        icp.align(final, T_map_base_guess.cast<float>());

        RCLCPP_INFO(this->get_logger(),
                    "ICP converged: %s",
                    icp.hasConverged() ? "true" : "false");
        RCLCPP_INFO(this->get_logger(), "ICP score: %f", icp.getFitnessScore());
        RCLCPP_INFO(this->get_logger(),
                    "ICP tranlation: %f, %f, %f",
                    icp.getFinalTransformation()(0, 3),
                    icp.getFinalTransformation()(1, 3),
                    icp.getFinalTransformation()(2, 3));

        // update T_map_odom
        if (icp.hasConverged() and icp.getFitnessScore() < 0.08 and
            icp.getFinalTransformation()(0, 3) < 0.1 and
            icp.getFinalTransformation()(1, 3) < 0.1 and
            icp.getFinalTransformation()(2, 3) == 0)
        {
            T_map_odom =
                T_map_odom * icp.getFinalTransformation().cast<double>();

            if (icp.getFitnessScore() < ICP_THRESHOLD)
            {
                pcl::PointCloud<pcl::PointXYZ> lastCloudInMap;
                pcl::transformPointCloud(lastCloud,
                                         lastCloudInMap,
                                         T_map_odom.cast<float>().inverse());

                mapCloud += lastCloud;

                // downsample
                pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
                voxel_grid.setInputCloud(mapCloud.makeShared());
                voxel_grid.setLeafSize(DOWN_SAMPLE_RESOLUTION,
                                       DOWN_SAMPLE_RESOLUTION,
                                       DOWN_SAMPLE_RESOLUTION);
                voxel_grid.filter(mapCloud);
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        (void)msg;
        geometry_msgs::msg::TransformStamped tf_base_odom;
        try
        {
            tf_base_odom = tf_buffer_->lookupTransform(
                "base_link", "odom", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        T_odom_base.setIdentity();
        T_odom_base.block<3, 1>(0, 3) << tf_base_odom.transform.translation.x,
            tf_base_odom.transform.translation.y,
            tf_base_odom.transform.translation.z;

        Eigen::Quaterniond q_odom_base(tf_base_odom.transform.rotation.w,
                                       tf_base_odom.transform.rotation.x,
                                       tf_base_odom.transform.rotation.y,
                                       tf_base_odom.transform.rotation.z);
        T_odom_base.block<3, 3>(0, 0) = q_odom_base.toRotationMatrix();
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // convert laser_scan to pcl point cloud
        pcl::PointCloud<pcl::PointXYZ> cloud;
        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            float range = msg->ranges[i];
            if (range < msg->range_min or range > msg->range_max)
            {
                continue;
            }
            float angle = msg->angle_min + i * msg->angle_increment;
            pcl::PointXYZ point;
            point.x = range * cos(angle);
            point.y = range * sin(angle);
            point.z = 0;
            cloud.push_back(point);
        }

        // downsample
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setInputCloud(cloud.makeShared());
        voxel_grid.setLeafSize(DOWN_SAMPLE_RESOLUTION,
                               DOWN_SAMPLE_RESOLUTION,
                               DOWN_SAMPLE_RESOLUTION);
        voxel_grid.filter(cloud);

        // transform to map frame
        geometry_msgs::msg::TransformStamped tf_odom_lidar;
        try
        {
            tf_odom_lidar = tf_buffer_->lookupTransform(
                "odom", msg->header.frame_id, tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        Eigen::Matrix4d T_odom_lidar;
        T_odom_lidar.setIdentity();
        T_odom_lidar.block<3, 1>(0, 3) << tf_odom_lidar.transform.translation.x,
            tf_odom_lidar.transform.translation.y,
            tf_odom_lidar.transform.translation.z;

        Eigen::Quaterniond q_odom_lidar(tf_odom_lidar.transform.rotation.w,
                                        tf_odom_lidar.transform.rotation.x,
                                        tf_odom_lidar.transform.rotation.y,
                                        tf_odom_lidar.transform.rotation.z);

        T_odom_lidar.block<3, 3>(0, 0) = q_odom_lidar.toRotationMatrix();

        Eigen::Matrix4d T_map_lidar = T_map_odom * T_odom_lidar;

        pcl::transformPointCloud(cloud, cloud, T_map_lidar.cast<float>());

        // add to map
        // mapCloud += cloud;
        if (mapCloud.empty())
        {
            mapCloud = cloud;
        }

        // publish map
        sensor_msgs::msg::PointCloud2 mapMsg;
        pcl::toROSMsg(mapCloud, mapMsg);
        mapMsg.header.frame_id = "map";
        mapMsg.header.stamp    = this->now();
        lastCloud              = cloud;
        map_pub_->publish(mapMsg);

        // broadcast tf
        geometry_msgs::msg::TransformStamped tf_map_odom;
        tf_map_odom.header.stamp            = msg->header.stamp;
        tf_map_odom.header.frame_id         = "map";
        tf_map_odom.child_frame_id          = "odom";
        tf_map_odom.transform.translation.x = T_map_odom(0, 3);
        tf_map_odom.transform.translation.y = T_map_odom(1, 3);
        tf_map_odom.transform.translation.z = T_map_odom(2, 3);
        Eigen::Quaterniond q_map_odom(T_map_odom.block<3, 3>(0, 0));
        tf_map_odom.transform.rotation.w = q_map_odom.w();
        tf_map_odom.transform.rotation.x = q_map_odom.x();
        tf_map_odom.transform.rotation.y = q_map_odom.y();
        tf_map_odom.transform.rotation.z = q_map_odom.z();
        tf_broadcaster_->sendTransform(tf_map_odom);
    }

    constexpr static double DOWN_SAMPLE_RESOLUTION = 0.05;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    Eigen::Matrix4d T_odom_base;
    Eigen::Matrix4d T_map_odom;

    pcl::PointCloud<pcl::PointXYZ> mapCloud;
    pcl::PointCloud<pcl::PointXYZ> lastCloud;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LidarLandmarker>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}