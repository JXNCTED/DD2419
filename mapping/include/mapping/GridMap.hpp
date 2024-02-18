#pragma once
#include <eigen3/Eigen/Core>
#include <opencv4/opencv2/opencv.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>
#include <rclcpp/rclcpp.hpp>

class GridMap
{
public:
    // constructors
    GridMap() = delete;
    GridMap(const double &gridSize, const int &sizeX, const int &sizeY, const int &startX, const int &startY);

    // getter and setter
    void setGridBelief(const double &x, const double &y, const double &belief);
    void setGridLogBelief(const double &x, const double &y, const double &logBelief);

    double getGridSize() { return gridSize; }
    double getGridLogBelief(const double &x, const double &y);
    // APIs
    nav_msgs::msg::OccupancyGrid toRosOccGrid(const std::string &frameId);

private:
    const double gridSize = 0.0;
    const int sizeX = 0, sizeY = 0;
    const int startX = 0, startY = 0;
    Eigen::MatrixXd gridBelief;
    rclcpp::Time lastUpdated;
};