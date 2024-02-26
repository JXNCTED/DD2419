#include "rclcpp/rclcpp.hpp"

class DetectionML : public rclcpp::Node
{
   public:
    DetectionML() : Node("detection_ml") {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionML>());
    rclcpp::shutdown();

    return 0;
}
