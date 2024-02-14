#include "mapping/GridMap.hpp"

GridMap::GridMap(const double &gridSize) : gridSize(gridSize)
{
    gridBelief.resize(sizeX, sizeY);
}

// nav_msg::OccupancyGrid GridMap::toRosOccGrid(const std::string &frameId)
// {
//     nav_msg::OccupancyGrid ret;
//     ret.header.frame_id = frameId;
//     ret.header.stamp = lastUpdated;

//     ret.info.width = sizeX;
//     ret.info.height = sizeY;
//     ret.info.resolution = gridSize;
//     ret.info.origin.position.x = -startX * gridSize;
//     ret.info.origin.position.y = -startY * gridSize;

//     for (size_t i = 0; i < sizeX * sizeY; i++)
//     {
//         const double &value = beliefGrid.data()[i];
//         if (value == 0.5)
//         {
//             ret.data.push_back(-1); // unknown
//         }
//         else
//         {
//             ret.data.push_back(value * 100);
//         }
//     }

//     return ret;
// }

void GridMap::setGridBelief(const double &x, const double &y, const double &belief)
{
    int xOnGrid = cvFloor(x / gridSize) + startX;
    int yOnGrid = cvFloor(y / gridSize) + startY;
    if (xOnGrid < 0 or xOnGrid > sizeX or yOnGrid < 0 or yOnGrid > sizeY)
    {
        return;
    }

    gridBelief(xOnGrid, yOnGrid) = belief;
}

void GridMap::setGridLogBelif(const double &x, const double &y, const double &logBelief)
{
    const double belief = 1.0f - 1.0f / (1 + exp(logBelief));
    setGridBelief(x, y, belief);
}