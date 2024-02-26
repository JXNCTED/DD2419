#include "mapping/Mapper.hpp"
#include "rcpputils/asserts.hpp"

Mapper::Mapper(GridMap *map) : map(map) {}

// occupancy probability
const double P_FREE  = 0.4;
const double P_PRIOR = 0.5;
const double P_OCC   = 0.7;

// LiDAR measurement model
double laserInvModel(const double &r, const double &R, const double &gridSize)
{
    // for all point less than ray measurement R, believe it's free
    if (r < (R - 0.5 * gridSize))
    {
        return P_FREE;
    }

    // for all point greater than ray measurement R, believe it's occupied
    if (r > (R + 0.5 * gridSize))
    {
        return P_PRIOR;
    }

    // for all point on the ray measurement R, believe it's occupied
    return P_OCC;
}

// update the gridmap with the laser scan
void Mapper::updateMapLaser(
    const sensor_msgs::msg::LaserScan::SharedPtr laserPtr,
    const Pose &robotPose)
{
    const double &gridSize = map->getGridSize();

    for (size_t i = 0; i < laserPtr->ranges.size(); i++)
    {
        const double R = laserPtr->ranges.at(i);
        // remove invalid measurement of INF
        if (R > laserPtr->range_max or R < laserPtr->range_min)
        {
            continue;
        }

        // calculate the angle of the laser
        const double angle =
            laserPtr->angle_increment * i + laserPtr->angle_min;
        const double cosAng = cos(angle);
        const double sinAng = sin(angle);

        Eigen::Vector2d lastPw(Eigen::Infinity, Eigen::Infinity);
        for (double r = 0; r < R + gridSize; r += gridSize)
        {
            // calculate the point in the local frame
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
            updateGrid(pW, occuProb);
            lastPw = pW;
        }
    }
}

void Mapper::updateGrid(const Eigen::Vector2d grid, const double &pOcc)
{
    // get the log belief of the grid
    double logBelief = map->getGridLogBelief(grid(0), grid(1));
    if (logBelief < 0)
    {
        return;  // error
    }
    // update the log belief
    logBelief += log(pOcc / (1 - pOcc));
    // set belief back
    map->setGridLogBelief(grid(0), grid(1), logBelief);
}
