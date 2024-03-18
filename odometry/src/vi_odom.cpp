#include <cv_bridge/cv_bridge.h>

#include "opencv4/opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "string"
#include "vector"

void drawImage(const cv::Mat &img, const std::string name)
{
    cv::imshow(name, img);
    cv::waitKey(1);
}
void drawImage(const cv::Mat &img, const std::vector<cv::Point2f> &pts, const std::string name)
{
    auto draw = img.clone();
    for (auto pt : pts)
    {
        cv::circle(draw, pt, 2, cv::Scalar(255, 0, 0), -1, 8);
    }

    cv::imshow(name, draw);
    cv::waitKey(1);
}

class VisualOdom : public rclcpp::Node
{
   public:
    VisualOdom() : Node("visual_odom")
    {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10, std::bind(&VisualOdom::imgCallback, this, std::placeholders::_1));
    }

    void imgCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        static std::vector<cv::Point2f> nPts;

        cv_bridge::CvImageConstPtr ptr;

        ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat filtered;
        cv::bilateralFilter(ptr->image, filtered, 3, 75, 75);
        drawImage(filtered, "filtered");
        cv::Mat canny;
        cv::Canny(filtered, canny, 80, 150, 3, true);
        cv::convertScaleAbs(canny, canny);
        drawImage(canny, "canny");

        cv::Mat dilated;
        cv::dilate(canny, dilated, cv::Mat(), cv::Point(-1, -1), 2, 1, 1);
        drawImage(dilated, "dilated");
        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarchy;

        cv::findContours(dilated, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat contoured = ptr->image.clone();

        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::drawContours(contoured, contours, (int)i, cv::Scalar(0, 255, 255), 2, cv::LINE_8, hierarchy, 0);
        }
        drawImage(contoured, "contoured");

        cv::goodFeaturesToTrack(filtered, nPts, 100, 0.01, 12.0, cv::Mat(), 3, false, 0.04);
        drawImage(ptr->image, nPts, "feature");
    }

   private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisualOdom>());

    rclcpp::shutdown();
    return 0;
}
