#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

const double YAW_THRESHOLD =
    0.15;  // only publish the lidar if yaw velocity is below this threshold
// only publish the lidar if yaw velocity is below this threshold
class LidarPassThrough : public rclcpp::Node
{
   public:
    LidarPassThrough() : Node("lidar_pass_through")
    {
        callback_group =
            this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        rclcpp::SubscriptionOptions imu_sub_options;
        imu_sub_options.callback_group = callback_group;

        imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data_raw",
            rclcpp::SensorDataQoS(),
            std::bind(
                &LidarPassThrough::imuCallback, this, std::placeholders::_1),
            imu_sub_options);
        rclcpp::SubscriptionOptions scan_sub_options;
        scan_sub_options.callback_group = callback_group;
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(
                &LidarPassThrough::scanCallback, this, std::placeholders::_1),
            scan_sub_options);

        valid_scan_pub = this->create_publisher<sensor_msgs::msg::LaserScan>(
            "/valid_scan", 10);
    }
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        last_stamp = msg->header.stamp;
        // a simple delay to ensure the imu data is stable
        static size_t stableCnt = 0;
        stableCnt = (std::abs(msg->angular_velocity.z) < YAW_THRESHOLD)
                        ? stableCnt + 1
                        : 0;
        is_stable = stableCnt > 70;
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx);
        if (is_stable and rclcpp::Time(msg->header.stamp).nanoseconds() >
                              last_stamp.nanoseconds())
        {
            valid_scan_pub->publish(*msg);
        }
    }

   private:
    bool is_stable = false;
    std::mutex mtx;
    rclcpp::Time last_stamp;

    rclcpp::CallbackGroup::SharedPtr callback_group;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr valid_scan_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPassThrough>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}