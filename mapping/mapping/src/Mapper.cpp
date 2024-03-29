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
static double laserInvModel(const double &r,
                            const double &R,
                            const double &gridSize)
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
        if (R > laserPtr->range_max or R < laserPtr->range_min)
        {
            R = laserPtr->range_max;
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

sensor_msgs::msg::PointCloud2 Mapper::updateMapRGBD(
    const sensor_msgs::msg::PointCloud2::SharedPtr rgbdPtr, const Pose &pose)
{
    // convert the point cloud to pcl point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*rgbdPtr, *cloud);

    // downsample the point cloud
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);

    // filter the point only at height -3.5 cm
    const double HEIGHT = 0.035;
    const double THRESH = 0.001;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFiltered(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    const double &gridSize = map->getGridSize();
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        const double R = sqrt(cloud->points[i].x * cloud->points[i].x +
                              cloud->points[i].z * cloud->points[i].z);
        if (cloud->points[i].y > HEIGHT - THRESH and
            cloud->points[i].y < HEIGHT + THRESH and R > 0.15 and R < 1.0)
        {
            cloudFiltered->points.push_back(cloud->points[i]);
            const double x      = cloud->points[i].z;
            const double y      = -cloud->points[i].x;
            const double angle  = atan2(y, x);
            const double cosAng = cos(angle);
            const double sinAng = sin(angle);

            Eigen::Vector2d lastPw(Eigen::Infinity, Eigen::Infinity);
            for (double r = 0; r < R + gridSize; r += gridSize)
            {
                Eigen::Vector2d pL(r * cosAng, r * sinAng);
                Eigen::Matrix2d rot;
                rot << cos(pose.theta), -sin(pose.theta), sin(pose.theta),
                    cos(pose.theta);
                Eigen::Vector2d t(pose.x, pose.y);
                Eigen::Vector2d pW = rot * pL + t;

                if (pW == lastPw)
                {
                    continue;
                }
                const double occuProb = laserInvModel(r, R, gridSize);
                updateGrid(pW, occuProb, GridMap::GridType::RGBD);
                lastPw = pW;
            }
        }
    }

    // return the filtered point cloud
    sensor_msgs::msg::PointCloud2 cloudMsg;
    pcl::toROSMsg(*cloudFiltered, cloudMsg);
    cloudMsg.header = rgbdPtr->header;

    return cloudMsg;
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
