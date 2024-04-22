#include "mapping/Mapper.hpp"

#include "pcl/filters/voxel_grid.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rcpputils/asserts.hpp"

Mapper::Mapper(GridMap *map) : map(map) {}

// occupancy probability
const double P_FREE = 0.4;
// unknown
const double P_PRIOR = 0.5;
const double P_OCC   = 0.7;

// LiDAR measurement model
/**
 * @brief calculate the inverse model of the laser measurement
 *
 * @param r distance along the scan ray
 * @param R distance at the end of the scan ray, hit an obstacle
 * @param gridSize size of the grid
 * @return double occupancy probability for this measurement
 */
static auto laserInvModel(const double &r,
                          const double &R,
                          const double &gridSize) -> double
{
    // for all point less than ray measurement R, believe it's free
    if (r < (R - 0.5 * gridSize))
    {
        return P_FREE;
    }

    // for all point greater than ray measurement R, believe it's unknown
    if (r > (R + 0.5 * gridSize))
    {
        return P_PRIOR;
    }

    // for all point on the ray measurement R, believe it's occupied
    return P_OCC;
}

void Mapper::updateMapLiDAR(
    const sensor_msgs::msg::LaserScan::SharedPtr laserPtr,
    const Pose &robotPose)
{
    const double &gridSize = map->getGridSize();

    // for all LiDAR measurements
    for (size_t i = 0; i < laserPtr->ranges.size(); i++)
    {
        double R = laserPtr->ranges.at(i);
        // remove invalid measurement of INF
        if (R > laserPtr->range_max)
        {
            // R = laserPtr->range_max;
            continue;
        }
        else if (R < laserPtr->range_min)
        {
            continue;
        }

        // calculate the angle of the laser
        const double angle =
            laserPtr->angle_increment * i + laserPtr->angle_min;
        const double cosAng = cos(angle);
        const double sinAng = sin(angle);

        // store the point last updated
        Eigen::Vector2d lastPw(Eigen::Infinity, Eigen::Infinity);
        for (double r = 0; r < R + gridSize; r += gridSize)
        {
            // calculate the point in the LiDAR frame
            Eigen::Vector2d pL(r * cosAng, r * sinAng);
            Eigen::Matrix2d rot;
            rot << cos(robotPose.theta), -sin(robotPose.theta),
                sin(robotPose.theta), cos(robotPose.theta);
            Eigen::Vector2d t(robotPose.x, robotPose.y);

            Eigen::Vector2d pW = rot * pL + t;

            if (pW == lastPw)
            {
                continue;
            }
            // get laser inverse model probability
            const double occuProb = laserInvModel(r, R, gridSize);
            // update with the inverse model
            updateGrid(pW, occuProb, GridMap::GridType::LiDAR);
            lastPw = pW;
        }
    }
}

void Mapper::updateMapLiDAR(
    const sensor_msgs::msg::PointCloud2::SharedPtr laserPtr,
    const Pose &robotPose)
{
    const double &gridSize = map->getGridSize();

    // convert the point cloud to pcl point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*laserPtr, *cloud);

    // for all points in the point cloud
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        const double x = cloud->points[i].x;
        const double y = cloud->points[i].y;

        // get the distance of the point
        const double R      = sqrt(x * x + y * y);
        const double angle  = atan2(y, x);
        const double cosAng = cos(angle);
        const double sinAng = sin(angle);

        // store the point last updated
        Eigen::Vector2d lastPw(Eigen::Infinity, Eigen::Infinity);
        for (double r = 0; r < R + gridSize; r += gridSize)
        {
            // calculate the point in the LiDAR frame
            Eigen::Vector2d pL(r * cosAng, r * sinAng);
            Eigen::Matrix2d rot;
            rot << cos(robotPose.theta), -sin(robotPose.theta),
                sin(robotPose.theta), cos(robotPose.theta);
            Eigen::Vector2d t(robotPose.x, robotPose.y);

            Eigen::Vector2d pW = rot * pL + t;

            if (pW == lastPw)
            {
                continue;
            }
            // get laser inverse model probability
            const double occuProb = laserInvModel(r, R, gridSize);
            // update with the inverse model
            updateGrid(pW, occuProb, GridMap::GridType::LiDAR);
            lastPw = pW;
        }
    }
}

void Mapper::updateMapRGBD(
    const sensor_msgs::msg::LaserScan::SharedPtr laserPtr,
    const Pose &robotPose)
{
    const double &gridSize = map->getGridSize();

    // for all LiDAR measurements
    for (size_t i = 0; i < laserPtr->ranges.size(); i++)
    {
        double R = laserPtr->ranges.at(i);
        // remove invalid measurement of INF
        if (R > laserPtr->range_max)
        {
            continue;
            // R = 100;
        }
        else if (R < laserPtr->range_min)
        {
            continue;
        }

        // calculate the angle of the laser
        const double angle =
            laserPtr->angle_increment * i + laserPtr->angle_min;
        const double cosAng = cos(angle);
        const double sinAng = sin(angle);

        // store the point last updated
        Eigen::Vector2d lastPw(Eigen::Infinity, Eigen::Infinity);
        for (double r = 0; r < R + gridSize; r += gridSize)
        {
            // calculate the point in the LiDAR frame
            Eigen::Vector2d pL(r * cosAng, r * sinAng);
            Eigen::Matrix2d rot;
            rot << cos(robotPose.theta), -sin(robotPose.theta),
                sin(robotPose.theta), cos(robotPose.theta);
            Eigen::Vector2d t(robotPose.x, robotPose.y);

            Eigen::Vector2d pW = rot * pL + t;

            if (pW == lastPw)
            {
                continue;
            }
            // get laser inverse model probability
            const double occuProb = laserInvModel(r, R, gridSize);
            // update with the inverse model
            updateGrid(pW, occuProb, GridMap::GridType::RGBD);
            lastPw = pW;
        }
    }
}

void Mapper::updateGrid(const Eigen::Vector2d coor,
                        const double &pOcc,
                        const GridMap::GridType &type)
{
    // get the log belief of the grid
    double logBelief = map->getGridLogBelief(coor(0), coor(1), type);
    if (logBelief < 0)
    {
        return;  // error
    }
    // update the log belief
    logBelief += log(pOcc / (1 - pOcc));
    // set belief back
    map->setGridLogBelief(coor(0), coor(1), logBelief, type);
}
