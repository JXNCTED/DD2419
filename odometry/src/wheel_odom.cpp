#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robp_interfaces/msg/encoders.hpp"

class WheelOdom : public rclcpp::Node
{
   public:
    WheelOdom() : Node("wheel_odom")
    {
        odom_pub_    = this->create_publisher<geometry_msgs::msg::Vector3>("/wheel_odom", 10);
        encoder_sub_ = this->create_subscription<robp_interfaces::msg::Encoders>(
            "/motor/encoders", 10, std::bind(&WheelOdom::encodersCallback, this, std::placeholders::_1));
    }

   private:
    void encodersCallback(const robp_interfaces::msg::Encoders::SharedPtr msg)
    {
        const float TICK_PER_REV = 3600;
        const float WHEEL_RADIUS = 0.05;
        const float WHEEL_BASE   = 0.3;

        const float DT = 50.0 / 1000.0;

        float vw1 = msg->delta_encoder_left * 2 * M_PI / TICK_PER_REV / DT;
        float vw2 = msg->delta_encoder_right * 2 * M_PI / TICK_PER_REV / DT;

        float v = (vw1 + vw2) * WHEEL_RADIUS / 2;
        float w = (vw2 - vw1) * WHEEL_RADIUS / WHEEL_BASE;

        geometry_msgs::msg::Vector3 odom;
        odom.x = v;
        odom.y = w;
        odom_pub_->publish(odom);
    }

    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr odom_pub_;
    rclcpp::Subscription<robp_interfaces::msg::Encoders>::SharedPtr encoder_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelOdom>());
    rclcpp::shutdown();
    return 0;
}
