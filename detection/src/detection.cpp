#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/filters/voxel_grid.h"
// #include "pcl/common/io.h"
#include "cmath"
#include "std_msgs/msg/float32_multi_array.hpp"

class Dectection : public rclcpp::Node
{
  struct BoundingBox
  {
    int x, y, w, h;
    float score;
    int class_id;
  };

public:
  Dectection() : Node("detection")
  {
    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 10, std::bind(&Dectection::cloud_callback, this, std::placeholders::_1));
    bbs_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/detection_ml/bounding_box", 10, std::bind(&Dectection::bbs_callback, this, std::placeholders::_1));

    downsampled_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/ds_points", 10);
    detected_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/camera/depth/color/detected_points", 10);
  }

private:
  void bbs_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    bbs.clear();
    for (size_t i = 0; i < msg->data.size(); i += 6)
    {
      BoundingBox bb;
      bb.x = msg->data[i];
      bb.y = msg->data[i + 1];
      bb.w = msg->data[i + 2];
      bb.h = msg->data[i + 3];
      bb.score = msg->data[i + 4];
      bb.class_id = msg->data[i + 5];
      bbs.push_back(bb);
    }
  }
  void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {

    // convert to pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    cloud->width = msg->width;
    cloud->height = msg->height;
    cloud->is_dense = false;
    cloud->points.resize(cloud->width * cloud->height);

    // downsampling filter and publish
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f);
    sor.filter(*cloud_filtered);
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered, output);
    output.header.frame_id = "camera_depth_optical_frame";
    downsampled_pub_->publish(output);

    // if no bounding boxes, return
    if (bbs.size() == 0)
    {
      return;
    }

    // filter the points inside the bounding boxes
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_detected(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto &bb : bbs)
    {
      for (int i = bb.x; i < bb.x + bb.w; i++)
      {
        for (int j = bb.y; j < bb.y + bb.h; j++)
        {
          if (i < 0 || i >= (int)cloud->width || j < 0 || j >= (int)cloud->height)
          {
            continue;
          }
          cloud_detected->push_back(cloud->at(i, j));
        }
      }
    }
    sensor_msgs::msg::PointCloud2 detected_msg;
    pcl::toROSMsg(*cloud_detected, detected_msg);
    detected_msg.header.frame_id = "camera_depth_optical_frame";
    detected_pub_->publish(detected_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr bbs_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr detected_pub_;

  std::vector<BoundingBox> bbs;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Dectection>());
  rclcpp::shutdown();
  return 0;
}
