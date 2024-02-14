#pragma once
#include "mapping/GridMap.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

struct Pose
{
    double x, y, theta;
};
class Mapper
{
public:
    // constructors
    Mapper() = delete;
    Mapper(GridMap *map);

    // APIs
    void updateMapLaser(const sensor_msgs::msg::LaserScan::SharedPtr laserPtr, const Pose &pose);

private:
    void updateGrid(const Eigen::Vector2d grid, const double &pOcc);
    GridMap *map;
};