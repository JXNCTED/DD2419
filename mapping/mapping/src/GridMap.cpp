#include "mapping/GridMap.hpp"

#include <fstream>

GridMap::GridMap(const double &gridSize,
                 const int &sizeX,
                 const int &sizeY,
                 const int &startX,
                 const int &startY)
    : gridSize(gridSize),
      sizeX(sizeX),
      sizeY(sizeY),
      startX(startX),
      startY(startY)
{
    gridBelief.resize(sizeX, sizeY);
    gridBelief.setOnes() *= 0.5;
    gridBeliefLiDAR.resize(sizeX, sizeY);
    gridBeliefLiDAR.setOnes() *= 0.5;
    gridBeliefRGBD.resize(sizeX, sizeY);
    gridBeliefRGBD.setOnes() *= 0.5;

    rosOccGrid.header.frame_id        = "map";
    rosOccGrid.info.width             = sizeX;
    rosOccGrid.info.height            = sizeY;
    rosOccGrid.info.resolution        = gridSize;
    rosOccGrid.info.origin.position.x = -startX * gridSize;
    rosOccGrid.info.origin.position.y = -startY * gridSize;
}

GridMap::GridMap(const std::string &dir)
{
    std::ifstream ifs(dir, std::ifstream::in);
    if (!ifs)
    {
        std::cerr << "Failed to open file: " << dir << std::endl;
        return;
    }
    ifs >> sizeX >> sizeY >> startX >> startY >> gridSize;
    gridBelief.resize(sizeX, sizeY);
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            ifs >> gridBelief(i, j);
        }
    }
    ifs.close();
}
nav_msgs::msg::OccupancyGrid GridMap::toRosOccGrid() { return rosOccGrid; }

void GridMap::setGridBelief(const double &x,
                            const double &y,
                            const double &belief)
{
    int xOnGrid = cvFloor(x / gridSize) + startX;
    int yOnGrid = cvFloor(y / gridSize) + startY;
    if (xOnGrid < 0 or xOnGrid >= sizeX or yOnGrid < 0 or yOnGrid >= sizeY)
    {
        return;
    }

    gridBelief(xOnGrid, yOnGrid) = belief;
    if (belief == 0.5f)
    {
        rosOccGrid.data[xOnGrid + yOnGrid * sizeX] = -1;
    }
    else
    {
        rosOccGrid.data[xOnGrid + yOnGrid * sizeX] = belief * 100;
    }
}

void GridMap::setGridLogBelief(const double &x,
                               const double &y,
                               const double &logBelief)
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

void GridMap::saveMap(const std::string &dir)
{
    std::ofstream ofs;
    ofs.open(dir);

    ofs << sizeX << " " << sizeY << " " << startX << " " << startY << " "
        << gridSize << std::endl;
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            ofs << gridBelief(i, j) << " ";
        }
        ofs << std::endl;
    }
    ofs.close();
}