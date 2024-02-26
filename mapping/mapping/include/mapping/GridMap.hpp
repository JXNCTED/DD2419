#pragma once
#include <eigen3/Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

class GridMap
{
   public:
    // constructors
    GridMap() = delete;
    GridMap(const double &gridSize,
            const int &sizeX,
            const int &sizeY,
            const int &startX,
            const int &startY);
    // constructor from file
    GridMap(const std::string &dir);

    // getter and setter
    void setGridBelief(const double &x, const double &y, const double &belief);
    void setGridLogBelief(const double &x,
                          const double &y,
                          const double &logBelief);

    double getGridSize() { return gridSize; }
    double getGridLogBelief(const double &x, const double &y);
    // APIs
    nav_msgs::msg::OccupancyGrid toRosOccGrid();
    void saveMap(const std::string &dir);

    nav_msgs::msg::Path planPath(const double &startX,
                                 const double &startY,
                                 const double &goalX,
                                 const double &goalY);

   private:
    std::vector<std::pair<int, int>> aStar(const int &startX,
                                           const int &startY,
                                           const int &goalX,
                                           const int &goalY);
    void expandGrid();
    void setOnesAroundPoint(const int &x, const int &y, const int &radius);
    nav_msgs::msg::OccupancyGrid rosOccGrid;
    double gridSize = 0.0;
    int sizeX = 0, sizeY = 0;
    int startX = 0, startY = 0;
    Eigen::MatrixXd gridBelief;
    Eigen::MatrixXi expandedGrid;
    //     cv::Mat expandedGridCV;
    //     Eigen::MatrixXd gridBeliefLiDAR;
    //     Eigen::MatrixXd gridBeliefRGBD;
    rclcpp::Time lastUpdated;
};