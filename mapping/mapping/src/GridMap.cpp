#include "mapping/GridMap.hpp"

#include <fstream>
#include <vector>
#include <thread>

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
    // gridBeliefLiDAR.resize(sizeX, sizeY);
    // gridBeliefLiDAR.setOnes() *= 0.5;
    // gridBeliefRGBD.resize(sizeX, sizeY);
    // gridBeliefRGBD.setOnes() *= 0.5;

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

struct Node
{
    int x, y;
    float g, h, f;
    Node *parent;
    Node(int x, int y, float g, float h, Node *parent)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent)
    {
    }
    bool operator<(const Node &rhs) const { return f > rhs.f; }
};

float heuristic(int x, int y, int goalX, int goalY)
{
    return sqrt((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY));
}

std::vector<std::pair<int, int>> GridMap::aStar(const int &startX,
                                                const int &startY,
                                                const int &goalX,
                                                const int &goalY)
{
    // return path
    std::vector<std::pair<int, int>> path;
    // openSet and closedSet
    std::priority_queue<Node> openSet;
    std::vector<std::vector<bool>> closedSet(sizeX,
                                             std::vector<bool>(sizeY, false));

    std::vector<std::vector<Node *>> nodes(sizeX,
                                           std::vector<Node *>(sizeY, nullptr));

    openSet.push(Node(
        startX, startY, 0, heuristic(startX, startY, goalX, goalY), nullptr));
    const int MAX_ITER = 1000;
    int iter           = 0;
    bool found         = false;
    expandGrid();

    while (!openSet.empty() and iter++ < MAX_ITER)
    {
        Node current = openSet.top();
        openSet.pop();
        if (current.x == goalX and current.y == goalY)
        {
            found = true;
            break;
        }
        closedSet[current.x][current.y] = true;
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int x = current.x + i;
                int y = current.y + j;
                if (x < 0 or x >= sizeX or y < 0 or y >= sizeY)
                {
                    continue;
                }
                if (closedSet[x][y])
                {
                    continue;
                }
                if (expandedGrid(x, y) == 1)  // if the grid is occupied
                {
                    continue;
                }
                float g = current.g + sqrt(i * i + j * j);
                float h = heuristic(x, y, goalX, goalY);
                if (nodes[x][y] == nullptr)
                {
                    nodes[x][y] = new Node(x, y, g, h, &current);
                    openSet.push(*nodes[x][y]);
                }
                else if (g < nodes[x][y]->g)
                {
                    nodes[x][y]->g      = g;
                    nodes[x][y]->f      = g + h;
                    nodes[x][y]->parent = &current;
                }
            }
        }
    }

    if (found)
    {
        Node *current = nodes[goalX][goalY];
        while (current != nullptr)
        {
            path.push_back(std::make_pair(current->x, current->y));
            current = current->parent;
        }
    }

    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            if (nodes[i][j] != nullptr)
            {
                delete nodes[i][j];
            }
        }
    }
    return path;
}

nav_msgs::msg::Path GridMap::planPath(const double &startX,
                                      const double &startY,
                                      const double &goalX,
                                      const double &goalY)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = rclcpp::Clock().now();

    int startXOnGrid = cvFloor(startX / gridSize) + this->startX;
    int startYOnGrid = cvFloor(startY / gridSize) + this->startY;
    int goalXOnGrid  = cvFloor(goalX / gridSize) + this->startX;
    int goalYOnGrid  = cvFloor(goalY / gridSize) + this->startY;

    std::vector<std::pair<int, int>> pathVec =
        aStar(startXOnGrid, startYOnGrid, goalXOnGrid, goalYOnGrid);
    if (pathVec.empty())
    {
        return path;
    }
    for (auto &p : pathVec)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = (p.first - this->startX) * gridSize;
        pose.pose.position.y = (p.second - this->startY) * gridSize;
        path.poses.push_back(pose);
    }

    return path;
}

void GridMap::expandGrid()
{
    expandedGrid.resize(sizeX, sizeY);
    expandedGrid.setZero();
    const int EXPAND_RADIUS = 1;
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            setOnesAroundPoint(i, j, EXPAND_RADIUS);
        }
    }
    cv::imshow("expandedGrid", expandedGridCV);
}

void GridMap::setOnesAroundPoint(const int &x, const int &y, const int &radius)
{
    for (int i = -radius; i <= radius; i++)
    {
        for (int j = -radius; j <= radius; j++)
        {
            int xOnGrid = x + i;
            int yOnGrid = y + j;
            if (xOnGrid < 0 or xOnGrid >= sizeX or yOnGrid < 0 or
                yOnGrid >= sizeY)
            {
                continue;
            }
            if (pow(i, 2) + pow(j, 2) <= pow(radius, 2))
            {
                expandedGrid(xOnGrid, yOnGrid)             = 1;
                expandedGridCV.at<uchar>(xOnGrid, yOnGrid) = 255;
            }
        }
    }
}