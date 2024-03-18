#include <deque>
#include <mutex>
#include <vector>

#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

constexpr size_t QUEUE_SIZE = 2000;
class LidarCompensator : public rclcpp::Node
{
   public:
    LidarCompensator() : Node("lidar_compensator")
    {
        callback_group =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions imu_sub_options;
        imu_sub_options.callback_group = callback_group;

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            rclcpp::SensorDataQoS(),
            std::bind(
                &LidarCompensator::imuCallback, this, std::placeholders::_1),
            imu_sub_options);
        rclcpp::SubscriptionOptions scan_sub_options;
        scan_sub_options.callback_group = callback_group;
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(
                &LidarCompensator::scanCallback, this, std::placeholders::_1),
            scan_sub_options);

        compensated_scan_pc_pub =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/compensated_scan_pc", 10);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_queue_mutex);
        imu_queue.push_back(*msg);
    }
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        static bool first_scan = true;
        // do not process the first scan
        if (first_scan)
        {
            first_scan = false;
            return;
        }
        scan_queue.push_back(*msg);
        // make sure have enough time
        if (scan_queue.size() < 2)
        {
            return;
        }
        // get the first scan
        sensor_msgs::msg::LaserScan scan = scan_queue.front();
        scan_queue.pop_front();

        // get start and end of scan time
        const double scanEndTime   = rclcpp::Time(scan.header.stamp).seconds();
        const double scanStartTime = rclcpp::Time(scan.header.stamp).seconds() -
                                     scan.time_increment * scan.ranges.size();

        // process imu queue
        if (not processImuQueue(scanStartTime, scanEndTime))
        {
            RCLCPP_WARN(this->get_logger(), "IMU prune failed");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> compensated_scan_pc_pcl;
        for (size_t i = 0; i < scan.ranges.size(); i++)
        {
            if (std::isinf(scan.ranges[i]) or scan.ranges[i] > scan.range_max or
                scan.ranges[i] < scan.range_min)
            {
                continue;
            }

            double angle = scan.angle_min + i * scan.angle_increment;
            double x     = scan.ranges[i] * std::cos(angle);
            double y     = scan.ranges[i] * std::sin(angle);
            double t     = scanStartTime + i * scan.time_increment;

            int imuIndex = (t - scanStartTime) / scan.time_increment;
            double rot   = 0.5 * (rotZ[imuIndex] + rotZ[imuIndex + 1]);
            if (rot < 0.0001)
            {
                rot = 0;
            }

            double xCompensated = x * std::cos(rot) - y * std::sin(rot);
            double yCompensated = x * std::sin(rot) + y * std::cos(rot);

            // hacky way to remove points that are too far from the original
            if (std::abs(x - xCompensated) > 0.1 or
                std::abs(y - yCompensated) > 0.1)
            {
                continue;
            }

            pcl::PointXYZ point;
            point.x = xCompensated;
            point.y = yCompensated;
            point.z = 0;
            compensated_scan_pc_pcl.push_back(point);
        }

        sensor_msgs::msg::PointCloud2 compensated_scan_pc;
        pcl::toROSMsg(compensated_scan_pc_pcl, compensated_scan_pc);
        compensated_scan_pc.header = scan.header;

        compensated_scan_pc_pub->publish(compensated_scan_pc);
    }

   private:
    bool processImuQueue(const double &scanStartTime, const double &scanEndTime)
    {
        std::lock_guard<std::mutex> lock(imu_queue_mutex);
        if (imu_queue.empty() or
            rclcpp::Time(imu_queue.front().header.stamp).seconds() >
                scanStartTime or
            rclcpp::Time(imu_queue.back().header.stamp).seconds() < scanEndTime)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for more IMU data");
            return false;
        }
        while (not imu_queue.empty() and
               rclcpp::Time(imu_queue.front().header.stamp).seconds() <
                   scanStartTime - 0.1)  // 0.1 second before scan start to make
                                         // sure we have enough imu data
        {
            imu_queue.pop_front();
        }

        if (imu_queue.empty())
        {
            RCLCPP_INFO(this->get_logger(), "IMU queue is empty after pruning");
            return false;
        }

        rotZ.clear();
        double lastTime =
            rclcpp::Time(imu_queue.front().header.stamp).seconds();
        for (size_t i = 0; i < imu_queue.size(); i++)
        {
            // if imu data is before scan start time, set rotation to 0
            if (rclcpp::Time(imu_queue[i].header.stamp).seconds() <
                scanStartTime)
            {
                rotZ.push_back(0);
                lastTime = rclcpp::Time(imu_queue[i].header.stamp).seconds();
            }
            // if imu data is after scan end time, calculate the last rotation
            // and break
            else if (rclcpp::Time(imu_queue[i].header.stamp).seconds() >
                     scanEndTime)
            {
                double dt = scanEndTime - lastTime;
                rotZ.push_back(imu_queue[i].angular_velocity.z * dt +
                               rotZ.back());
                break;
            }
            // if imu data is between scan start and end time, calculate the
            // rotation
            else
            {
                double dt = rclcpp::Time(imu_queue[i].header.stamp).seconds() -
                            lastTime;
                lastTime = rclcpp::Time(imu_queue[i].header.stamp).seconds();
                rotZ.push_back(imu_queue[i].angular_velocity.z * dt +
                               rotZ.back());
            }
        }
        return true;
    }
    rclcpp::CallbackGroup::SharedPtr callback_group;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        compensated_scan_pc_pub;

    std::vector<double> rotZ;  // rotation in radians in yaw direction
    std::deque<sensor_msgs::msg::Imu> imu_queue;
    std::deque<sensor_msgs::msg::LaserScan> scan_queue;
    std::mutex imu_queue_mutex;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCompensator>();
    auto executor =
        std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    rclcpp::shutdown();
    return 0;
}