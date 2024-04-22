/**
 * @file mapping.cpp
 * @author ZOU Hetai
 * @brief the node for mapping and path planning
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <fstream>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/transform_stamped__struct.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mapping/GridMap.hpp"
#include "mapping/Mapper.hpp"
#include "mapping_interfaces/srv/path_plan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class MappingNode : public rclcpp::Node
{
   public:
    MappingNode()
        : Node("mapping"), map(0.025, 250, 500, 180, 200), mapper(&map)
    {
        // bunch of pubs and subs
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/valid_scan",
            10,
            std::bind(
                &MappingNode::lidarCallback, this, std::placeholders::_1));

        rclcpp::SensorDataQoS qos;
        point_cloud_sub_ =
            this->create_subscription<sensor_msgs::msg::LaserScan>(
                "/realsense_scan",
                qos,
                std::bind(&MappingNode::pointCloudCallback,
                          this,
                          std::placeholders::_1));

        occu_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/occupancy", 10);

        // read from /home/group7/workspace_2_tsv.tsv
        std::ifstream file("/home/group7/workspace_2_tsv.tsv", std::ios::in);
        std::string line;
        std::vector<std::pair<double, double>> lineSegments;
        std::getline(file, line);  // skip the first line
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            double x, y;
            if (!(iss >> x >> y))
            {
                break;
            }
            lineSegments.push_back(std::make_pair(x, y));
        }
        map.setLineSegmentOccupied(lineSegments);
        file.close();
        // tf2
        tfBuffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        RCLCPP_INFO(this->get_logger(), "Mapping node initialized");
    }
    void pointCloudCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        mapper.updateMapRGBD(msg, pose_camera);
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        (void)msg;
        try
        {
            transform_stamped_map_lidar = tfBuffer->lookupTransform(
                "map", "lidar_link", tf2::TimePointZero);
            transform_stamped_map_camera = tfBuffer->lookupTransform(
                "map", "camera_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }

        pose_camera.x = transform_stamped_map_camera.transform.translation.x;
        pose_camera.y = transform_stamped_map_camera.transform.translation.y;
        tf2::Quaternion q(transform_stamped_map_camera.transform.rotation.x,
                          transform_stamped_map_camera.transform.rotation.y,
                          transform_stamped_map_camera.transform.rotation.z,
                          transform_stamped_map_camera.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose_camera.theta = yaw;

        pose_lidar.x = transform_stamped_map_lidar.transform.translation.x;
        pose_lidar.y = transform_stamped_map_lidar.transform.translation.y;
        tf2::Quaternion q2(transform_stamped_map_lidar.transform.rotation.x,
                           transform_stamped_map_lidar.transform.rotation.y,
                           transform_stamped_map_lidar.transform.rotation.z,
                           transform_stamped_map_lidar.transform.rotation.w);
        tf2::Matrix3x3 m2(q2);
        m2.getRPY(roll, pitch, yaw);
        pose_lidar.theta = yaw;
    }

    // get LIDAR measurement and call the updateMapLaser function
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // mapper.updateMapLiDAR(msg, pose);
        (void)msg;
        nav_msgs::msg::OccupancyGrid occu = map.toRosOccGrid();
        occu_pub_->publish(occu);
    }
    auto getMap() -> const GridMap & { return map; }

   private:
    GridMap map;
    Mapper mapper;
    Pose pose_lidar;   // pose lidar in map
    Pose pose_camera;  // pose camera in map
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        point_cloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::TransformStamped transform_stamped_map_lidar;
    geometry_msgs::msg::TransformStamped transform_stamped_map_camera;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;
};

std::shared_ptr<MappingNode> node;

/**
 * @brief path plan service handle function, request and response are defined in
 * mapping_interfaces/srv/PathPlan
 *
 * @param request
 * @param response
 */
void planPath(
    const std::shared_ptr<mapping_interfaces::srv::PathPlan::Request> request,
    std::shared_ptr<mapping_interfaces::srv::PathPlan::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("planPath"), "Incoming request");
    GridMap &map = const_cast<GridMap &>(node->getMap());

    response->path = map.planPath(request->current_pose.pose.position.x,
                                  request->current_pose.pose.position.y,
                                  request->goal_pose.pose.position.x,
                                  request->goal_pose.pose.position.y);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    node = std::make_shared<MappingNode>();
    rclcpp::Service<mapping_interfaces::srv::PathPlan>::SharedPtr service =
        node->create_service<mapping_interfaces::srv::PathPlan>("path_plan",
                                                                planPath);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}