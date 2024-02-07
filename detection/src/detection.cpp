#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"

class Dectection : public rclcpp::Node
{
public:
  Dectection() : Node("detection")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&Dectection::cloud_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/ds_points", 10);
    detected_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/detected_points", 10);
  }

private:
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "camera_depth_optical_frame";
    pub_->publish(output);

    sensor_msgs::msg::PointCloud2 detected_output;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected(new pcl::PointCloud<pcl::PointXYZRGB>);

    // filter red points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_red(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZRGB point = cloud->points[i];
      float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (point.r > 200 && point.g < 150 && point.b < 150 && dist < 1.5)
      {
        cloud_filtered_red->points.push_back(point);
        cloud_detected->points.push_back(point);
      }
    }

    if (cloud_filtered_red->points.size() > 0)
    {
      float avgCoord[3] = {0};
      for (size_t i = 0; i < cloud_filtered_red->points.size(); i++)
      {
        pcl::PointXYZRGB point = cloud_filtered_red->points[i];
        avgCoord[0] += point.x;
        avgCoord[1] += point.y;
        avgCoord[2] += point.z;
      }
      avgCoord[0] /= cloud_filtered_red->points.size();
      avgCoord[1] /= cloud_filtered_red->points.size();
      avgCoord[2] /= cloud_filtered_red->points.size();

      RCLCPP_INFO(this->get_logger(), "\x1b[31m RED at distance: %f \x1b[0m", sqrt(avgCoord[0] * avgCoord[0] + avgCoord[1] * avgCoord[1] + avgCoord[2] * avgCoord[2]));
    }

    // filter green points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_green(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
      pcl::PointXYZRGB point = cloud->points[i];

      float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if (50 < point.r && point.r < 68 && 110 < point.g && point.g < 130 && 110 < point.b && point.b < 130 && dist < 1.5)
      {
        // RCLCPP_INFO(this->get_logger(), "R: %d, G: %d, B: %d, dist: %f", point.r, point.g, point.b, dist);
        cloud_filtered_green->points.push_back(point);
        cloud_detected->points.push_back(point);
        // RCLCPP_INFO(this->get_logger(), "\x1b[32m GREEN at distance: %f \x1b[0m", sqrt(avgCoord[0] * avgCoord[0] + avgCoord[1] * avgCoord[1] + avgCoord[2] * avgCoord[2]));
      }
    }
    if (cloud_filtered_green->points.size() > 0)
    {
      float avgCoord[3] = {0};
      for (size_t i = 0; i < cloud_filtered_green->points.size(); i++)
      {
        pcl::PointXYZRGB point = cloud_filtered_green->points[i];
        avgCoord[0] += point.x;
        avgCoord[1] += point.y;
        avgCoord[2] += point.z;
      }
      avgCoord[0] /= cloud_filtered_green->points.size();
      avgCoord[1] /= cloud_filtered_green->points.size();
      avgCoord[2] /= cloud_filtered_green->points.size();

      RCLCPP_INFO(this->get_logger(), "\x1b[32m GREEN at distance: %f \x1b[0m", sqrt(avgCoord[0] * avgCoord[0] + avgCoord[1] * avgCoord[1] + avgCoord[2] * avgCoord[2]));
    }

    pcl::toROSMsg(*cloud_detected, detected_output);
    detected_output.header.frame_id = "camera_depth_optical_frame";
    detected_pub_->publish(detected_output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dectection>());
  rclcpp::shutdown();
  return 0;
}
