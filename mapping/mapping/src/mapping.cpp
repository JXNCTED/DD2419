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
#include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>

#include "detection_interfaces/msg/box_list.hpp"
#include "detection_interfaces/msg/stuff_list.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "mapping/GridMap.hpp"
#include "mapping/Mapper.hpp"
#include "mapping_interfaces/srv/get_frontier.hpp"
#include "mapping_interfaces/srv/path_plan.hpp"
#include "mapping_interfaces/srv/path_plan_box.hpp"
#include "mapping_interfaces/srv/path_plan_object.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std::chrono_literals;

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

        plan_path_pub_ =
            this->create_publisher<nav_msgs::msg::Path>("/plan_path", 10);

        path_plan_target_pub_ =
            this->create_publisher<geometry_msgs::msg::PointStamped>(
                "/path_plan_target", 10);

        stuff_list_sub_ =
            this->create_subscription<detection_interfaces::msg::StuffList>(
                "/category_eval/stuff_list",
                10,
                std::bind(&MappingNode::stuffListCallback,
                          this,
                          std::placeholders::_1));
        box_list_sub_ =
            this->create_subscription<detection_interfaces::msg::BoxList>(
                "/box_list",
                10,
                std::bind(&MappingNode::stuffBoxCallback,
                          this,
                          std::placeholders::_1));

        timer_ = this->create_wall_timer(500ms,
                                         [this]()
                                         {
                                             nav_msgs::msg::OccupancyGrid occu =
                                                 map.toRosOccGrid();
                                             occu_pub_->publish(occu);
                                         });

        servicePlanPath =
            this->create_service<mapping_interfaces::srv::PathPlan>(
                "path_plan",
                std::bind(&MappingNode::planPath,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));

        servicePlanPathObject =
            this->create_service<mapping_interfaces::srv::PathPlanObject>(
                "path_plan_object",
                std::bind(&MappingNode::planPathObject,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));

        servicePlanPathBox =
            this->create_service<mapping_interfaces::srv::PathPlanBox>(
                "path_plan_box",
                std::bind(&MappingNode::planPathBox,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));

        serviceGetFrontier =
            this->create_service<mapping_interfaces::srv::GetFrontier>(
                "get_frontier",
                std::bind(&MappingNode::getFrontier,
                          this,
                          std::placeholders::_1,
                          std::placeholders::_2));

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
            lineSegments.emplace_back(x, y);
        }

        map.setLineSegmentOccupied(lineSegments);
        file.close();
        // tf2
        tfBuffer   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tfListener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

        RCLCPP_INFO(this->get_logger(), "Mapping node initialized");
    }

    void planPath(
        const std::shared_ptr<mapping_interfaces::srv::PathPlan::Request>
            request,
        std::shared_ptr<mapping_interfaces::srv::PathPlan::Response> response)
    {
        RCLCPP_INFO(rclcpp::get_logger("planPath"), "Incoming request");

        geometry_msgs::msg::TransformStamped transform_stamped_map_odom;

        geometry_msgs::msg::PointStamped target_point;
        target_point.point           = request->goal_pose.pose.position;
        target_point.header.frame_id = "map";
        target_point.header.stamp    = this->now();

        path_plan_target_pub_->publish(target_point);

        // geometry_msgs::msg::TransformStamped transform_stamped_odom_map;
        tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(odom_msg_->header.stamp.sec) +
            std::chrono::nanoseconds(odom_msg_->header.stamp.nanosec));
        try
        {
            transform_stamped_map_odom = tfBuffer->lookupTransform(
                "map", "odom", time_point, tf2::durationFromSec(0.8));
            // transform_stamped_odom_map = tfBuffer->lookupTransform(
            //     "odom", "map", time_point, tf2::durationFromSec(0.8));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }

        geometry_msgs::msg::Pose start_pose;
        tf2::doTransform(
            odom_msg_->pose.pose, start_pose, transform_stamped_map_odom);

        response->path                 = map.planPath(start_pose.position.x,
                                      start_pose.position.y,
                                      request->goal_pose.pose.position.x,
                                      request->goal_pose.pose.position.y);
        response->path.header.frame_id = "map";
        response->path.header.stamp    = this->now();

        plan_path_pub_->publish(response->path);
    }

    void getFrontier(
        const std::shared_ptr<mapping_interfaces::srv::GetFrontier::Request>
            request,
        std::shared_ptr<mapping_interfaces::srv::GetFrontier::Response>
            response)
    {
        (void)request;
        RCLCPP_INFO(rclcpp::get_logger("getFrontier"), "Incoming request");

        auto frontier = map.getFrontier();
        // pick a random frontier
        if (frontier.size() > 0)
        {
            auto randomIndex  = rand() % frontier.size();
            response->success = true;
            response->p.x     = frontier[randomIndex].first;
            response->p.y     = frontier[randomIndex].second;
            response->p.z     = 0.0;
        }
        else
        {
            response->success = false;
        }
    }
    void planPathBox(
        const std::shared_ptr<mapping_interfaces::srv::PathPlanBox::Request>
            request,
        std::shared_ptr<mapping_interfaces::srv::PathPlanBox::Response>
            response)
    {
        RCLCPP_INFO(rclcpp::get_logger("planPathBox"), "Incoming request");

        geometry_msgs::msg::TransformStamped transform_stamped_map_odom;
        // geometry_msgs::msg::TransformStamped transform_stamped_odom_map;
        tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(odom_msg_->header.stamp.sec) +
            std::chrono::nanoseconds(odom_msg_->header.stamp.nanosec));
        try
        {
            transform_stamped_map_odom = tfBuffer->lookupTransform(
                "map", "odom", time_point, tf2::durationFromSec(0.8));
            // transform_stamped_odom_map = tfBuffer->lookupTransform(
            //     "odom", "map", time_point, tf2::durationFromSec(0.8));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }

        geometry_msgs::msg::Pose start_pose;
        tf2::doTransform(
            odom_msg_->pose.pose, start_pose, transform_stamped_map_odom);

        response->path                 = map.planPathBox(start_pose.position.x,
                                         start_pose.position.y,
                                         request->target_box_id);
        response->path.header.frame_id = "map";
        response->path.header.stamp    = this->now();

        plan_path_pub_->publish(response->path);
    }

    void planPathObject(
        const std::shared_ptr<mapping_interfaces::srv::PathPlanObject::Request>
            request,
        std::shared_ptr<mapping_interfaces::srv::PathPlanObject::Response>
            response)
    {
        RCLCPP_INFO(rclcpp::get_logger("planPathObject"), "Incoming request");

        geometry_msgs::msg::TransformStamped transform_stamped_map_odom;
        // geometry_msgs::msg::TransformStamped transform_stamped_odom_map;
        tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(odom_msg_->header.stamp.sec) +
            std::chrono::nanoseconds(odom_msg_->header.stamp.nanosec));
        try
        {
            transform_stamped_map_odom = tfBuffer->lookupTransform(
                "map", "odom", time_point, tf2::durationFromSec(0.8));
            // transform_stamped_odom_map = tfBuffer->lookupTransform(
            //     "odom", "map", time_point, tf2::durationFromSec(0.8));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }

        geometry_msgs::msg::Pose start_pose;
        tf2::doTransform(
            odom_msg_->pose.pose, start_pose, transform_stamped_map_odom);

        response->path                 = map.planPath(start_pose.position.x,
                                      start_pose.position.y,
                                      request->target_object_id);
        response->path.header.frame_id = "map";
        response->path.header.stamp    = this->now();

        plan_path_pub_->publish(response->path);
    }

    void stuffListCallback(
        const detection_interfaces::msg::StuffList::SharedPtr msg)
    {
        mapper.updateMapStuffList(msg);
    }

    void stuffBoxCallback(
        const detection_interfaces::msg::BoxList::SharedPtr msg)
    {
        mapper.updateMapBoxList(msg);
    }

    void pointCloudCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        try
        {
            transform_stamped_map_camera =
                tfBuffer->lookupTransform("map",
                                          "camera_depth_frame",
                                          msg->header.stamp,
                                          tf2::durationFromSec(0.3));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
            return;
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

        mapper.updateMapRGBD(msg, pose_camera);
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg_ = msg;
        tf2::TimePoint time_point =
            tf2::TimePoint(std::chrono::seconds(msg->header.stamp.sec) +
                           std::chrono::nanoseconds(msg->header.stamp.nanosec));
        try
        {
            transform_stamped_map_lidar = tfBuffer->lookupTransform(
                "map", "lidar_link", time_point, tf2::durationFromSec(0.8));
            transform_stamped_map_camera =
                tfBuffer->lookupTransform("map",
                                          "camera_depth_frame",
                                          time_point,
                                          tf2::durationFromSec(0.3));
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
        }
    }

    // get LIDAR measurement and call the updateMapLaser function
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // mapper.updateMapLiDAR(msg, pose);
        (void)msg;
        // nav_msgs::msg::OccupancyGrid occu = map.toRosOccGrid();
        // occu_pub_->publish(occu);
    }
    auto getMap() -> const GridMap & { return map; }

   private:
    GridMap map;
    Mapper mapper;
    /* Pose pose_lidar;   // pose lidar in map */
    Pose pose_camera;  // pose camera in map
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        point_cloud_sub_;
    rclcpp::Subscription<detection_interfaces::msg::StuffList>::SharedPtr
        stuff_list_sub_;
    rclcpp::Subscription<detection_interfaces::msg::BoxList>::SharedPtr
        box_list_sub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occu_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr plan_path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr
        path_plan_target_pub_;

    geometry_msgs::msg::TransformStamped transform_stamped_map_lidar;
    geometry_msgs::msg::TransformStamped transform_stamped_map_camera;

    std::unique_ptr<tf2_ros::Buffer> tfBuffer;
    std::shared_ptr<tf2_ros::TransformListener> tfListener;

    rclcpp::Service<mapping_interfaces::srv::PathPlan>::SharedPtr
        servicePlanPath;
    rclcpp::Service<mapping_interfaces::srv::PathPlanObject>::SharedPtr
        servicePlanPathObject;
    rclcpp::Service<mapping_interfaces::srv::PathPlanBox>::SharedPtr
        servicePlanPathBox;
    rclcpp::Service<mapping_interfaces::srv::GetFrontier>::SharedPtr
        serviceGetFrontier;
};

std::shared_ptr<MappingNode> node;

auto main(int argc, char **argv) -> int
{
    rclcpp::init(argc, argv);

    node = std::make_shared<MappingNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
