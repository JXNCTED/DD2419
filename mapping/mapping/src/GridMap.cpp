#include "mapping/GridMap.hpp"

#include <fstream>
#include <vector>

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
    rosOccGrid.data.resize(sizeX * sizeY);
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
    rosOccGrid.header.stamp      = rclcpp::Clock().now();
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

/**
 * @brief A start heuristic euler
 *
 * @param x current x
 * @param y current y
 * @param goalX goal x
 * @param goalY goal y
 * @return int heuristic value
 */
static inline float h(const int &x,
                      const int &y,
                      const int &goalX,
                      const int &goalY)
{
    return sqrt((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY));
}

static std::vector<std::pair<int, int>> aStar(const int &startX,
                                              const int &startY,
                                              const int &goalX,
                                              const int &goalY,
                                              const Eigen::MatrixXd &grid)
{
    std::vector<std::pair<int, int>> openSet;
    std::vector<std::pair<int, int>> closeSet;
    std::vector<std::pair<int, int>> path;
    openSet.push_back(std::make_pair(startX, startY));

    std::vector<std::vector<int>> cameFrom(grid.rows(),
                                           std::vector<int>(grid.cols(), -1));
    std::vector<std::vector<float>> gScore(
        grid.rows(), std::vector<float>(grid.cols(), FLT_MAX));
    std::vector<std::vector<float>> fScore(
        grid.rows(), std::vector<float>(grid.cols(), FLT_MAX));

    gScore[startX][startY] = 0;
    fScore[startX][startY] = h(startX, startY, goalX, goalY);

    while (!openSet.empty())
    {
        std::pair<int, int> current;
        float minFScore = FLT_MAX;
        for (auto &p : openSet)
        {
            if (fScore[p.first][p.second] < minFScore)
            {
                minFScore = fScore[p.first][p.second];
                current   = p;
            }
        }

        if (current.first == goalX and current.second == goalY)
        {
            while (current.first != startX or current.second != startY)
            {
                path.push_back(current);
                current = std::make_pair(
                    cameFrom[current.first][current.second], current.second);
            }
            path.push_back(std::make_pair(startX, startY));
            std::reverse(path.begin(), path.end());
            return path;
        }

        openSet.erase(std::remove(openSet.begin(), openSet.end(), current),
                      openSet.end());
        closeSet.push_back(current);

        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 and j == 0)
                {
                    continue;
                }
                int x = current.first + i;
                int y = current.second + j;
                if (x < 0 or x >= grid.rows() or y < 0 or y >= grid.cols())
                {
                    continue;
                }
                if (grid(x, y) == 0.5 or grid(x, y) > 0.8)
                {
                    continue;
                }
                if (std::find(closeSet.begin(),
                              closeSet.end(),
                              std::make_pair(x, y)) != closeSet.end())
                {
                    continue;
                }
                float tentativeGScore = gScore[current.first][current.second] +
                                        h(current.first, current.second, x, y);
                if (std::find(openSet.begin(),
                              openSet.end(),
                              std::make_pair(x, y)) == openSet.end())
                {
                    openSet.push_back(std::make_pair(x, y));
                }
                else if (tentativeGScore >= gScore[x][y])
                {
                    continue;
                }
                cameFrom[x][y] = current.first;
                gScore[x][y]   = tentativeGScore;
                fScore[x][y]   = gScore[x][y] + h(x, y, goalX, goalY);

                if (std::find(openSet.begin(),
                              openSet.end(),
                              std::make_pair(x, y)) == openSet.end())
                {
                    openSet.push_back(std::make_pair(x, y));
                }
            }
        }
    }
    RCLCPP_ERROR(rclcpp::get_logger("path plan"), "No path found");
    return path;
}

nav_msgs::msg::Path GridMap::planPath(const int &startX,
                                      const int &startY,
                                      const int &goalX,
                                      const int &goalY)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = rclcpp::Clock().now();

    std::vector<std::pair<int, int>> pathVec =
        aStar(startX, startY, goalX, goalY, gridBelief);
    for (auto &p : pathVec)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = (p.first - startX) * gridSize;
        pose.pose.position.y = (p.second - startY) * gridSize;
        path.poses.push_back(pose);
    }

    return path;
}