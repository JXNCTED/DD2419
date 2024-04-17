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
            3s, std::bind(&LidarLandmarker::timerCallback, this));
    }

   private:
    void timerCallback()
    {
        // inital guess
        Eigen::Matrix4d guess;
        guess.setIdentity();

        guess = base_T_odom * odom_T_map;

        // ICP
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(lastCloud.makeShared());
        icp.setInputTarget(mapCloud.makeShared());
        pcl::PointCloud<pcl::PointXYZ> cloud;
        icp.align(cloud, guess.cast<float>());
        if (!icp.hasConverged())
        {
            RCLCPP_WARN(this->get_logger(), "ICP did not converge");
            return;
        }

        Eigen::Matrix4f transformation = icp.getFinalTransformation();

        // update odom_T_map
        // Eigen::Affine3d odom_T_map_new;
        // odom_T_map_new.matrix() =
        //     transformation.cast<double>() * odom_T_map.matrix();

        // geometry_msgs::msg::TransformStamped odom_tf_map_new;
        // odom_tf_map_new.header.stamp    = this->now();
        // odom_tf_map_new.header.frame_id = "odom";
        // odom_tf_map_new.child_frame_id  = "map";
        // odom_tf_map_new.transform.translation.x =
        //     odom_T_map_new.translation().x();
        // odom_tf_map_new.transform.translation.y =
        //     odom_T_map_new.translation().y();
        // odom_tf_map_new.transform.translation.z =
        //     odom_T_map_new.translation().z();
        // // odom_tf_map_new.transform.rotation.w =
        // odom_T_map_new.rotation().w(); Eigen::Quaterniond
        // q_new(odom_T_map_new.rotation());
        // odom_tf_map_new.transform.rotation.w = q_new.w();
        // odom_tf_map_new.transform.rotation.x = q_new.x();
        // odom_tf_map_new.transform.rotation.y = q_new.y();
        // odom_tf_map_new.transform.rotation.z = q_new.z();
        Eigen::Matrix4d odom_T_map_new;
        odom_T_map_new = odom_T_map * transformation.cast<double>();

        geometry_msgs::msg::TransformStamped odom_tf_map_new;
        odom_tf_map_new.header.stamp            = this->now();
        odom_tf_map_new.header.frame_id         = "odom";
        odom_tf_map_new.child_frame_id          = "map";
        odom_tf_map_new.transform.translation.x = odom_T_map_new(0, 3);
        odom_tf_map_new.transform.translation.y = odom_T_map_new(1, 3);
        odom_tf_map_new.transform.translation.z = odom_T_map_new(2, 3);
        Eigen::Quaterniond q_new(odom_T_map_new.block<3, 3>(0, 0));
        odom_tf_map_new.transform.rotation.w = q_new.w();
        odom_tf_map_new.transform.rotation.x = q_new.x();
        odom_tf_map_new.transform.rotation.y = q_new.y();
        odom_tf_map_new.transform.rotation.z = q_new.z();

        odom_T_map = odom_T_map_new;

        tf_broadcaster_->sendTransform(odom_tf_map_new);
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        (void)msg;
        // get transform from odom to base_link
        geometry_msgs::msg::TransformStamped odom_tf_base;
        try
        {
            odom_tf_base = tf_buffer_->lookupTransform(
                "odom", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
        }

        // convert to Eigen homogeneous matrix
        Eigen::Matrix4d odom_T_base;
        odom_T_base.block<3, 1>(0, 3) << odom_tf_base.transform.translation.x,
            odom_tf_base.transform.translation.y,
            odom_tf_base.transform.translation.z;
        Eigen::Quaterniond q(odom_tf_base.transform.rotation.w,
                             odom_tf_base.transform.rotation.x,
                             odom_tf_base.transform.rotation.y,
                             odom_tf_base.transform.rotation.z);
        odom_T_base.block<3, 3>(0, 0) = q.toRotationMatrix();
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        static bool first = false;
        if (not first)
        {
            first = true;
            return;
        }

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
        voxel_grid.setLeafSize(0.1, 0.1, 0.1);
        voxel_grid.filter(cloud);

        // add to map
        mapCloud += cloud;

        // publish map
        sensor_msgs::msg::PointCloud2 mapMsg;
        pcl::toROSMsg(mapCloud, mapMsg);
        mapMsg.header.frame_id = "map";
        mapMsg.header.stamp    = this->now();
        lastCloud              = cloud;
        map_pub_->publish(mapMsg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

    Eigen::Matrix4d base_T_odom;
    Eigen::Matrix4d odom_T_map;

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