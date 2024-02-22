#include "mapping/GridMap.hpp"
#include <fstream>

GridMap::GridMap(const double &gridSize, const int &sizeX, const int &sizeY, const int &startX, const int &startY) : gridSize(gridSize), sizeX(sizeX), sizeY(sizeY),
                                                                                                                     startX(startX), startY(startY)
{
    gridBelief.resize(sizeX, sizeY);
    gridBelief.setOnes() *= 0.5;
}

nav_msgs::msg::OccupancyGrid GridMap::toRosOccGrid(const std::string &frameId)
{
    nav_msgs::msg::OccupancyGrid ret;
    ret.header.frame_id = frameId;
    ret.header.stamp = lastUpdated;

    ret.info.width = sizeX;
    ret.info.height = sizeY;
    ret.info.resolution = gridSize;
    ret.info.origin.position.x = -startX * gridSize;
    ret.info.origin.position.y = -startY * gridSize;

    for (int i = 0; i < sizeX * sizeY; i++)
    {
        const double &value = gridBelief.data()[i];
        if (value == 0.5f)
        {
            ret.data.push_back(-1); // unknown
        }
        else
        {
            ret.data.push_back(value * 100);
        }
    }

    return ret;
}

void GridMap::setGridBelief(const double &x, const double &y, const double &belief)
{
    int xOnGrid = cvFloor(x / gridSize) + startX;
    int yOnGrid = cvFloor(y / gridSize) + startY;
    if (xOnGrid < 0 or xOnGrid >= sizeX or yOnGrid < 0 or yOnGrid >= sizeY)
    {
        return;
    }

    gridBelief(xOnGrid, yOnGrid) = belief;
}

void GridMap::setGridLogBelief(const double &x, const double &y, const double &logBelief)
{
    const double belief = 1.0f - 1.0f / (1 + exp(logBelief));
    setGridBelief(x, y, belief);
}

double GridMap::getGridLogBelief(const double &x, const double &y)
{

    int xOnGrid = cvFloor(x / gridSize) + startX;
    int yOnGrid = cvFloor(y / gridSize) + startY;
    if (xOnGrid < 0 or xOnGrid >= sizeX or yOnGrid < 0 or yOnGrid >= sizeY)
    {
        return -1.0;
    }
    double belief = gridBelief(xOnGrid, yOnGrid);
    return log(belief / (1.0 - belief));
}

void GridMap::saveMap(const std::string dir &dir)
{
    std::ofstream ofs(dir, std::ofstream::out);
    ofs << sizeX << " " ofs << sizeY << " " ofs << startX << " " ofs << startY << std::endl;
    for (size_t i = 0; i < sizeX; i++)
    {
        for (size_t j = 0; j < sizeY; j++)
        {
            ofs << gridBelief(i, j) << " ";
        }
    }
    ofs.close();
}
void GridMap::loadMap(const std::string dir &dir)
{
}