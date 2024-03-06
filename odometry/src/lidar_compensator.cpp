#include <deque>
#include <mutex>
#include <vector>

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
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            10,
            std::bind(
                &LidarCompensator::imuCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            std::bind(
                &LidarCompensator::scanCallback, this, std::placeholders::_1));
        compensated_scan_pc_pub_ =
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
        std::lock_guard<std::mutex> lock(imu_queue_mutex);
        if (imu_queue.empty())
        {
            RCLCPP_INFO(this->get_logger(), "waiting for more IMU data");
            return;
        }
        // const double scanStartTime =
        // rclcpp::Time(msg->header.stamp).seconds(); const double scanEndTime =
        //     scanStartTime + msg->ranges.size() * msg->time_increment;
        const double scanEndTime = rclcpp::Time(msg->header.stamp).seconds();
        const double scanStartTime =
            scanEndTime - msg->ranges.size() * msg->time_increment;

        while (!imu_queue.empty())
        {
            const double imuTime =
                rclcpp::Time(imu_queue.front().header.stamp).seconds();
            // remove old IMU data
            if (imuTime < scanStartTime)
            {
                imu_queue.pop_front();
            }
            else
            {
                break;
            }
        }
        if (imu_queue.empty())
        {
            RCLCPP_WARN(this->get_logger(), "IMU data is too old");
            return;
        }

        rotZ.clear();
        for (size_t i = 0; i < imu_queue.size(); i++)
        {
            const double imuTime =
                rclcpp::Time(imu_queue[i].header.stamp).seconds();
            if (imuTime > scanEndTime)
            {
                break;
            }
            if (i == 0)
            {
                rotZ.push_back(0);
            }
            else
            {
                const double dt =
                    imuTime -
                    rclcpp::Time(imu_queue[i - 1].header.stamp).seconds();
                const double dtheta = 0.5 *
                                      (imu_queue[i].angular_velocity.z +
                                       imu_queue[i - 1].angular_velocity.z) *
                                      dt;
                rotZ.push_back(rotZ.back() + dtheta);
            }
            RCLCPP_INFO(this->get_logger(), "rotZ: %f", rotZ.back());
        }

        sensor_msgs::msg::PointCloud2 compensated_scan_pc;
        compensated_scan_pc.header = msg->header;
        compensated_scan_pc.height = 1;
        compensated_scan_pc.width  = msg->ranges.size();
        compensated_scan_pc.fields.resize(3);
        compensated_scan_pc.fields[0].name   = "x";
        compensated_scan_pc.fields[0].offset = 0;
        compensated_scan_pc.fields[0].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
        compensated_scan_pc.fields[0].count  = 1;
        compensated_scan_pc.fields[1].name   = "y";
        compensated_scan_pc.fields[1].offset = 4;
        compensated_scan_pc.fields[1].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
        compensated_scan_pc.fields[1].count  = 1;
        compensated_scan_pc.fields[2].name   = "z";
        compensated_scan_pc.fields[2].offset = 8;
        compensated_scan_pc.fields[2].datatype =
            sensor_msgs::msg::PointField::FLOAT32;
        compensated_scan_pc.fields[2].count = 1;
        compensated_scan_pc.is_bigendian    = false;
        compensated_scan_pc.point_step      = 16;
        compensated_scan_pc.row_step =
            compensated_scan_pc.point_step * compensated_scan_pc.width;
        compensated_scan_pc.is_dense = true;
        compensated_scan_pc.data.resize(compensated_scan_pc.row_step *
                                        compensated_scan_pc.height);

        for (size_t i = 0; i < msg->ranges.size(); i++)
        {
            const double range = msg->ranges[i];
            if (range < msg->range_min || range > msg->range_max)
            {
                continue;
            }
            const double scanTime = scanStartTime + i * msg->time_increment;
            int rotZIdx           = (scanTime - scanStartTime) /
                          (scanEndTime - scanStartTime) * rotZ.size();
            if (rotZIdx < 0)
            {
                rotZIdx = 0;
            }
            else if (rotZIdx >= rotZ.size())
            {
                rotZIdx = rotZ.size() - 1;
            }

            const double angle =
                msg->angle_min + i * msg->angle_increment - rotZ[rotZIdx];
            const double x = range * cos(angle);
            const double y = range * sin(angle);
            std::memcpy(
                &compensated_scan_pc.data[i * compensated_scan_pc.point_step +
                                          compensated_scan_pc.fields[0].offset],
                &x,
                sizeof(float));
            std::memcpy(
                &compensated_scan_pc.data[i * compensated_scan_pc.point_step +
                                          compensated_scan_pc.fields[1].offset],
                &y,
                sizeof(float));
        }
        compensated_scan_pc_pub_->publish(compensated_scan_pc);
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        compensated_scan_pc_pub_;

    std::vector<double> rotZ;  // rotation in radians in yaw direction
    std::deque<sensor_msgs::msg::Imu> imu_queue;
    std::mutex imu_queue_mutex;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCompensator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}