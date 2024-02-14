#pragma once
#include "mapping/GridMap.hpp"
// #include "sensor_msgs/msg/laser_scan.hpp"
// #include "geometry_msgs/msg/pose2_d.hpp"

class Mapper
{
public:
    // constructors
    Mapper() = delete;
    Mapper(GridMap *map);

    // APIs
    // void updateMapLaser(const sensor_msgs::LaserScan::SharedPtr laserPtr, const geometry_msgs::Pose2d::SharedPtr posePtr);

private:
    GridMap *map;
};