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

#include "mapping/GridMap.hpp"
#include "mapping/Mapper.hpp"
#include "mapping_interfaces/srv/path_plan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class MappingNode : public rclcpp::Node
{
   public:
    MappingNode() : Node("mapping"), map(0.05, 500, 500, 250, 250), mapper(&map)
    {
        // bunch of pubs and subs
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/compensated_scan_pc",
            10,
            std::bind(
                &MappingNode::lidarCallback, this, std::placeholders::_1));

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
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        pose.x = msg->pose.pose.position.x;
        pose.y = msg->pose.pose.position.y;
        tf2::Quaternion q(msg->pose.pose.orientation.x,
                          msg->pose.pose.orientation.y,
                          msg->pose.pose.orientation.z,
                          msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        pose.theta = yaw;
    }

    // get LIDAR measurement and call the updateMapLaser function
    void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        mapper.updateMapLiDAR(msg, pose);
        nav_msgs::msg::OccupancyGrid occu;
        occu = map.toRosOccGrid();
        occu_pub_->publish(occu);
    }

    const GridMap &getMap() { return map; }

   private:
    GridMap map;
    Mapper mapper;
    Pose pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
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