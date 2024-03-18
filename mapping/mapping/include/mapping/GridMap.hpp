/**
 * @file GridMap.hpp
 * @author ZOU Hetai
 * @brief GridMap class for storing the obstacle map
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <eigen3/Eigen/Core>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

/**
 * @brief GridMap class for storing the obstacle map
 *
 */
class GridMap
{
   public:
    // constructors
    GridMap() = delete;
    /**
     * @brief Construct a new Grid Map object based on the given parameters
     *
     * @param gridSize size of grid in meters
     * @param sizeX width of the grid in number of cells
     * @param sizeY height of the grid in number of cells
     * @param startX starting coordinate of the robot within the grid
     * @param startY starting coordinate of the robot within the grid
     */
    GridMap(const double &gridSize,
            const int &sizeX,
            const int &sizeY,
            const int &startX,
            const int &startY);
    // constructor from file
    /**
     * @brief Construct a new Grid Map object based on the given file
     *
     * @param dir directory of the file
     */
    GridMap(const std::string &dir);

    // getter and setter
    /**
     * @brief set belief in the grid
     *
     * @param x x coordinate
     * @param y y coordinate
     * @param belief probability of occupancy
     */
    void setGridBelief(const double &x, const double &y, const double &belief);
    /**
     * @brief Set log belief in the grid
     *
     * @param x x coordinate
     * @param y y coordinate
     * @param logBelief log probability of occupancy
     */
    void setGridLogBelief(const double &x,
                          const double &y,
                          const double &logBelief);

    /**
     * @brief Get the Grid Size object
     *
     * @return double grid cell size
     */
    double getGridSize() { return gridSize; }
    /**
     * @brief Get the log belief
     *
     * @param x x coordinate
     * @param y y coordinate
     * @return double log probability of occupancy
     */
    double getGridLogBelief(const double &x, const double &y);
    // APIs
    /**
     * @brief get the ros message of the occupancy grid
     *
     * @return nav_msgs::msg::OccupancyGrid ros message of the occupancy grid
     */
    nav_msgs::msg::OccupancyGrid toRosOccGrid();
    /**
     * @brief save the map to the given directory
     *
     * @param dir directory to save the map
     */
    void saveMap(const std::string &dir);

    // plan path in map coordinate
    /**
     * @brief Plan a path from start to goal in the map
     *
     * @param startX start coordinate in map frame
     * @param startY start coordinate in map frame
     * @param goalX goal coordinate in map frame
     * @param goalY goal coordinate in map frame
     * @return nav_msgs::msg::Path
     */
    nav_msgs::msg::Path planPath(const double &startX,
                                 const double &startY,
                                 const double &goalX,
                                 const double &goalY);

    /**
     * @brief Set the occupancy for the workspace
     *
     * @param lineSegments line segments to be set as occupied, in a pair of
     * coordinates
     */
    void setLineSegmentOccupied(
        const std::vector<std::pair<double, double>> &lineSegments);

   private:
    // a star algorithm
    /**
     * @brief   A* algorithm for path planning
     *
     * @param startX start X on grid coordinate
     * @param startY start Y on grid coordinate
     * @param goalX goal X on grid coordinate
     * @param goalY goal Y on grid coordinate
     * @return std::vector<std::pair<int, int>> a path a series of points
     */
    std::vector<std::pair<int, int>> aStar(const int &startX,
                                           const int &startY,
                                           const int &goalX,
                                           const int &goalY);
    /**
     * @brief Expand the grid to c-space
     *
     */
    void expandGrid(const float &radius = 0.3f);
    // helper function for expandGrid, set obstacles around a point
    /**
     * @brief Set every points within the radius of the given point as occupied
     *
     * @param x point of coordinate on grid
     * @param y point of coordinate on grid
     * @param radius radius of the circle
     */
    void setOnesAroundPoint(const int &x, const int &y, const int &radius);
    // ros message of the occupancy grid
    nav_msgs::msg::OccupancyGrid rosOccGrid;
    // size of the grid
    double gridSize = 0.0;
    int sizeX = 0, sizeY = 0;
    int startX = 0, startY = 0;
    // belief of the grid, occupancy grid, lidar only for now
    Eigen::MatrixXd gridBelief;
    // expanded grid
    Eigen::MatrixXi expandedGrid;
    // known obstacles
    Eigen::MatrixXi knownGrid;
    // not implemented yet, for RGBD and expanded grid
    //     cv::Mat expandedGridCV;
    //     Eigen::MatrixXd gridBeliefLiDAR;
    //     Eigen::MatrixXd gridBeliefRGBD;
    rclcpp::Time lastUpdated;
};