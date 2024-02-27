#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
// #include "pcl/common/io.h"
#include "std_msgs/msg/float32.hpp"
#include "cmath"

class Dectection : public rclcpp::Node
{
public:
  Dectection() : Node("detection")
  {
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&Dectection::cloud_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/ds_points", 10);
    theta_pub_ = this->create_publisher<std_msgs::msg::Float32>("/camera/depth/color/target_theta", 10);
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
    sor.setLeafSize(0.01f, 0.01f, 0.01f);

    sor.filter(*cloud_filtered);

    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "camera_depth_optical_frame";
    pub_->publish(output);

    const float DIST_THRES = 1.5f;
    // filter  points
    const float R_CENTER[] = {150, 40, 20};
    const float R_THRES = 45.0f;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_red(new pcl::PointCloud<pcl::PointXYZRGB>);

    const float G_CENTER[] = {0, 70, 60};
    const float G_THRES = 40.0f;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_green(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (size_t i = 0; i < cloud_filtered->points.size(); i++)
    {
      pcl::PointXYZRGB point = cloud_filtered->points[i];
      float dist = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
      if ((pow(point.r - R_CENTER[0], 2) + pow(point.g - R_CENTER[1], 2) + pow(point.b - R_CENTER[2], 2)) < pow(R_THRES, 2) && dist < DIST_THRES)
      {
        cloud_filtered_red->points.push_back(point);
        // cloud_detected->points.push_back(point);
      }

      if ((pow(point.r - G_CENTER[0], 2) + pow(point.g - G_CENTER[1], 2) + pow(point.b - G_CENTER[2], 2)) < pow(G_THRES, 2) && dist < DIST_THRES)
      {
        cloud_filtered_green->points.push_back(point);
        // cloud_detected->points.push_back(point);
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_filtered_red(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_red;
    sor_red.setInputCloud(cloud_filtered_red);
    sor_red.setMeanK(5);
    sor_red.setStddevMulThresh(0.5);
    sor_red.filter(*sor_filtered_red);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sor_filtered_green(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_green;
    sor_red.setInputCloud(cloud_filtered_green);
    sor_red.setMeanK(5);
    sor_red.setStddevMulThresh(1.0);
    sor_red.filter(*sor_filtered_green);

    if (sor_filtered_red->points.size() > 0)
    {
      float avgCoord[3] = {0};
      for (size_t i = 0; i < sor_filtered_red->points.size(); i++)
      {
        pcl::PointXYZRGB point = sor_filtered_red->points[i];
        avgCoord[0] += point.x;
        avgCoord[1] += point.y;
        avgCoord[2] += point.z;
      }
      avgCoord[0] /= sor_filtered_red->points.size();
      avgCoord[1] /= sor_filtered_red->points.size();
      avgCoord[2] /= sor_filtered_red->points.size();

      float theta = std::atan(avgCoord[0] / avgCoord[2]);

      std_msgs::msg::Float32 thetaMsg;
      thetaMsg.data = theta;

      theta_pub_->publish(thetaMsg);
    }

    float dist;
    if (sor_filtered_green->points.size() > 0)
    {
      float avgCoord[3] = {0};
      for (size_t i = 0; i < sor_filtered_green->points.size(); i++)
      {
        pcl::PointXYZRGB point = sor_filtered_green->points[i];
        avgCoord[0] += point.x;
        avgCoord[1] += point.y;
        avgCoord[2] += point.z;
      }
      avgCoord[0] /= sor_filtered_green->points.size();
      avgCoord[1] /= sor_filtered_green->points.size();
      avgCoord[2] /= sor_filtered_green->points.size();

      dist = sqrt(avgCoord[0] * avgCoord[0] + avgCoord[1] * avgCoord[1] + avgCoord[2] * avgCoord[2]);
    }

    std::cout << "Distance: " << dist << std::endl;

    sensor_msgs::msg::PointCloud2 detected_output;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected(new pcl::PointCloud<pcl::PointXYZRGB>);
    *cloud_detected = *sor_filtered_red + *sor_filtered_green;

    pcl::toROSMsg(*cloud_detected, detected_output);
    detected_output.header.frame_id = "camera_depth_optical_frame";
    detected_pub_->publish(detected_output);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr theta_pub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dectection>());
  rclcpp::shutdown();
  return 0;
}
