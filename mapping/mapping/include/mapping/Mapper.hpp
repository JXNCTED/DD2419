/**
 * @file Mapper.hpp
 * @author ZOU Hetai
 * @brief Mapper class for the mapping algorithm
 * @version 0.1
 * @date 2024-03-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "mapping/GridMap.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// pose struct
struct Pose
{
    // x, y, and yaw in meters and radians
    double x, y, theta;
};

/**
 * @brief Mapper class for updating the grid map with data
 *
 */
class Mapper
{
   public:
    // constructors
    Mapper() = delete;
    Mapper(GridMap *map);

    // APIs
    /**
     * @brief Update the map with the laser scan data
     *
     * @param laserPtr Ptr to the laser scan message
     * @param pose current pose of the robot in map frame
     */
    void updateMapLiDAR(const sensor_msgs::msg::PointCloud2::SharedPtr laserPtr,
                        const Pose &pose);

    // ovloaded function for easy testing
    void updateMapLiDAR(const sensor_msgs::msg::LaserScan::SharedPtr laserPtr,
                        const Pose &pose);

    /**
     * @brief Update the map with the RGBD data
     *
     * @param rgbdPtr Ptr to the RGBD message
     * @param pose current pose of the robot in map frame
     */
    // void updateMapRGBD(const sensor_msgs::msg::PointCloud2::SharedPtr
    // rgbdPtr,
    //                    const Pose &pose);
    void updateMapRGBD(const sensor_msgs::msg::LaserScan::SharedPtr rgbdPtr,
                       const Pose &pose);

   private:
    /**
     * @brief update the grid with the given occupancy probability
     *
     * @param coor coordinate to be updated
     * @param pOcc occupancy probability
     */
    void updateGrid(const Eigen::Vector2d coor,
                    const double &pOcc,
                    const GridMap::GridType &type);

    // pointer to the grid map, initialized in the constructor
    GridMap *map;
};