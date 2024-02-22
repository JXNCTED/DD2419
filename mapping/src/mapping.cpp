#include "mapping/GridMap.hpp"
#include "mapping/Mapper.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
class MappingNode : public rclcpp::Node
{
   public:
    MappingNode()
        : Node("mapping"), map(0.05, 1000, 1000, 500, 500), mapper(&map)
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&MappingNode::odomCallback, this, std::placeholders::_1));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(
                &MappingNode::laserCallback, this, std::placeholders::_1));
        occu_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/occupancy", 10);
        // save the map per 10s
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10000),
            std::bind(&MappingNode::timerCallback, this));
    }

    void timerCallback()
    {
        static int count = 0;
        map.saveMap("/home/group7/maps/" + std::to_string(count) + "_map.txt");
        count++;
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

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        mapper.updateMapLaser(msg, pose);
        nav_msgs::msg::OccupancyGrid occu;
        occu = map.toRosOccGrid("odom");
        occu_pub_->publish(occu);
    }

   private:
    GridMap map;
    Mapper mapper;
    Pose pose;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MappingNode>());

    rclcpp::shutdown();
    return 0;
}
