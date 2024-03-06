#include "rclcpp/rclcpp.hpp"

class LidarCompensator : public rclcpp::Node
{
   public:
    LidarCompensator() : Node("lidar_compensator")
    {
        ;
        ;
    }

   private:
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarCompensator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}