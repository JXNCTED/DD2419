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

    // plan path in map coordinate
    nav_msgs::msg::Path planPath(const double &startX,
                                 const double &startY,
                                 const double &goalX,
                                 const double &goalY);

    void setLineSegmentOccupied(
        const std::vector<std::pair<double, double>> &lineSegments);

   private:
    // a star algorithm
    std::vector<std::pair<int, int>> aStar(const int &startX,
                                           const int &startY,
                                           const int &goalX,
                                           const int &goalY);
    // expand to c-space
    void expandGrid();
    // helper function for expandGrid, set obstacles around a point
    void setOnesAroundPoint(const int &x, const int &y, const int &radius);
    nav_msgs::msg::OccupancyGrid rosOccGrid;
    // size of the grid
    double gridSize = 0.0;
    int sizeX = 0, sizeY = 0;
    int startX = 0, startY = 0;
    // belief of the grid, occupancy grid
    Eigen::MatrixXd gridBelief;
    Eigen::MatrixXi expandedGrid;
    Eigen::MatrixXi knownGrid;
    //     cv::Mat expandedGridCV;
    //     Eigen::MatrixXd gridBeliefLiDAR;
    //     Eigen::MatrixXd gridBeliefRGBD;
    rclcpp::Time lastUpdated;
};