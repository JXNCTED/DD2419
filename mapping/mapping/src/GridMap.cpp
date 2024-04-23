#include "mapping/GridMap.hpp"

#include <assert.h>

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
    knownGrid.resize(sizeX, sizeY);
    knownGrid.setZero();
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

    expandedGrid.resize(sizeX, sizeY);
}

auto GridMap::toRosOccGrid() -> nav_msgs::msg::OccupancyGrid
{
    // const double OCC_THRESHOLD = 0.7;
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            // set known grid, i.e. workspace
            if (knownGrid(i, j) == 1)
            {
                rosOccGrid.data[i + j * sizeX] = 100;
            }
            if (gridBeliefRGBD(i, j) == 0.5)
            {
                rosOccGrid.data[i + j * sizeX] = -1;
            }
            else
            {
                rosOccGrid.data[i + j * sizeX] = 100 * gridBeliefRGBD(i, j);
            }
            // if (gridBeliefLiDAR(i, j) >= OCC_THRESHOLD or
            //     gridBeliefRGBD(i, j) >= OCC_THRESHOLD)
            // {
            //     rosOccGrid.data[i + j * sizeX] = 100;
            // }
            // else if (gridBeliefLiDAR(i, j) == 0.5 and
            //          gridBeliefRGBD(i, j) == 0.5)
            // {
            //     rosOccGrid.data[i + j * sizeX] = -1;
            // }
        }
    }
    return rosOccGrid;
}

void GridMap::setGridBelief(const double &x,
                            const double &y,
                            const double &belief,
                            const GridType &type)
{
    // convert from coordinate to grid index
    int xOnGrid = cvFloor(x / gridSize) + startX;
    int yOnGrid = cvFloor(y / gridSize) + startY;
    if (xOnGrid < 0 or xOnGrid >= sizeX or yOnGrid < 0 or yOnGrid >= sizeY)
    {
        return;
    }

    // set the belief
    switch (type)
    {
    case GridType::LiDAR:
        gridBeliefLiDAR(xOnGrid, yOnGrid) = belief;
        break;
    case GridType::RGBD:
        gridBeliefRGBD(xOnGrid, yOnGrid) = belief;
        break;
    default:
        assert(false);
    }
    // update the ros message also
    rosOccGrid.header.stamp = rclcpp::Clock().now();
}

void GridMap::setGridLogBelief(const double &x,
                               const double &y,
                               const double &logBelief,
                               const GridType &type)
{
    const double belief = 1.0f - 1.0f / (1 + exp(logBelief));
    // const double beliefClamped = std::clamp(belief, 0.6, 0.9);
    // const double beliefClamped = std::clamp(belief, 0.1, 0.9);
    const double beliefClamped = std::clamp(belief, 0.0, 1.0);
    setGridBelief(x, y, beliefClamped, type);
}

auto GridMap::getGridLogBelief(const double &x,
                               const double &y,
                               const GridType &type) -> double
{
    int xOnGrid = cvFloor(x / gridSize) + startX;
    int yOnGrid = cvFloor(y / gridSize) + startY;
    if (xOnGrid < 0 or xOnGrid >= sizeX or yOnGrid < 0 or yOnGrid >= sizeY)
    {
        return -1.0;
    }
    // double belief = gridBelief(xOnGrid, yOnGrid);
    double belief = 0.0;
    switch (type)
    {
    case GridType::LiDAR:
        belief = gridBeliefLiDAR(xOnGrid, yOnGrid);
        break;
    case GridType::RGBD:
        belief = gridBeliefRGBD(xOnGrid, yOnGrid);
        break;
    default:
        assert(false);
    }
    return log(belief / (1.0 - belief));
}

/**
 * @brief Node struct for A* algorithm
 *
 */
struct Node
{
    int x = 0, y = 0;
    // g is the cost from start to current node
    // h is the heuristic cost from current node to goal
    // f is the total cost
    float g = 0, h = 0, f = 0;
    Node *parent = nullptr;
    Node(int x, int y, float g, float h, Node *parent)
        : x(x), y(y), g(g), h(h), f(g + h), parent(parent)
    {
    }
    // for priority queue
    auto operator<(const Node &rhs) const -> bool { return f > rhs.f; }
};

// euclidean distance
auto heuristic(int x, int y, int goalX, int goalY) -> float
{
    return sqrt((x - goalX) * (x - goalX) + (y - goalY) * (y - goalY));
}

auto GridMap::aStar(const int &startX,
                    const int &startY,
                    const int &goalX,
                    const int &goalY) -> std::vector<std::pair<int, int>>
{
    RCLCPP_INFO(
        rclcpp::get_logger("GridMap::aStar"), "current %d, %d", startX, startY);

    RCLCPP_INFO(
        rclcpp::get_logger("GridMap::aStar"), "goal %d, %d", goalX, goalY);
    // return path
    std::vector<std::pair<int, int>> path;
    // openSet and closedSet
    std::priority_queue<Node> openSet;
    std::vector<std::vector<bool>> closedSet(sizeX,
                                             std::vector<bool>(sizeY, false));

    std::vector<std::vector<Node *>> nodes(sizeX,
                                           std::vector<Node *>(sizeY, nullptr));

    // push the start node to the openSet
    openSet.push(Node(
        startX, startY, 0, heuristic(startX, startY, goalX, goalY), nullptr));
    // limit a max iter for safety
    const int MAX_ITER = 10000;
    int iter           = 0;
    bool found         = false;
    // expand the grid to c-space

    while (!openSet.empty() and iter++ < MAX_ITER)
    {
        if (iter % 1000 == 0)
        {
            // debug print to make sure things are running and not
            // stuck/infinite
            RCLCPP_INFO(rclcpp::get_logger("GridMap::aStar"),
                        "iter %d, openSet size %ld",
                        iter,
                        openSet.size());
        }
        Node current = openSet.top();
        openSet.pop();

        if (current.x == goalX and current.y == goalY)
        {
            RCLCPP_INFO(rclcpp::get_logger("GridMap::aStar"), "path found");
            found = true;
            break;
        }
        closedSet[current.x][current.y] = true;
        // for all neighbors
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                int x = current.x + i;
                int y = current.y + j;
                if (i == 0 and j == 0)
                {
                    continue;
                }
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
                    nodes[x][y] =
                        new Node(x, y, g, h, nodes[current.x][current.y]);
                    openSet.push(*nodes[x][y]);
                }
                else if (g < nodes[x][y]->g)
                {
                    nodes[x][y]->g      = g;
                    nodes[x][y]->f      = g + h;
                    nodes[x][y]->h      = h;
                    nodes[x][y]->parent = nodes[current.x][current.y];
                }
            }
        }
    }

    if (found)
    {
        Node *current = nodes[goalX][goalY];
        int cnt       = 0;
        // trace back the path
        while (current != nullptr and cnt++ < 1000)  // sanity check
        {
            path.push_back(std::make_pair(current->x, current->y));
            current = current->parent;
        }
        RCLCPP_INFO(rclcpp::get_logger("GridMap::aStar"),
                    "path found with length %ld",
                    path.size());
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("GridMap::aStar"), "path not found");
    }

    // free the memory
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

auto GridMap::planPath(const double &startX,
                       const double &startY,
                       const double &goalX,
                       const double &goalY) -> nav_msgs::msg::Path
{
    // just wrap the A* basically
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = rclcpp::Clock().now();

    int startXOnGrid = cvFloor(startX / gridSize) + this->startX;
    int startYOnGrid = cvFloor(startY / gridSize) + this->startY;
    int goalXOnGrid  = cvFloor(goalX / gridSize) + this->startX;
    int goalYOnGrid  = cvFloor(goalY / gridSize) + this->startY;

    RCLCPP_INFO(rclcpp::get_logger("GridMap::planPath"), "received, planning");

    expandGrid();
    std::vector<std::pair<int, int>> pathVec =
        aStar(startXOnGrid, startYOnGrid, goalXOnGrid, goalYOnGrid);
    if (pathVec.empty())
    {
        return path;
    }
    for (auto &p : pathVec)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp       = rclcpp::Clock().now();
        pose.header.frame_id    = "map";
        pose.pose.position.x    = (p.first - this->startX) * gridSize;
        pose.pose.position.y    = (p.second - this->startY) * gridSize;
        pose.pose.position.z    = 0;
        pose.pose.orientation.x = 1;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 0;

        path.poses.push_back(pose);
    }

    return path;
}

auto GridMap::planPath(const double &startX,
                       const double &startY,
                       const int &goalObjId) -> nav_msgs::msg::Path
{
    // just wrap the A* basically
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp    = rclcpp::Clock().now();

    int startXOnGrid = cvFloor(startX / gridSize) + this->startX;
    int startYOnGrid = cvFloor(startY / gridSize) + this->startY;
    int goalXOnGrid  = stuffList[goalObjId].first;
    int goalYOnGrid  = stuffList[goalObjId].second;

    RCLCPP_INFO(rclcpp::get_logger("GridMap::planPath (obj)"),
                "received target obj %d, goal %d, %d",
                goalObjId,
                goalXOnGrid,
                goalYOnGrid);

    expandGrid();

    std::vector<std::pair<int, int>> pathVec =
        aStar(startXOnGrid, startYOnGrid, goalXOnGrid, goalYOnGrid);

    if (pathVec.empty())
    {
        return path;
    }

    for (auto &p : pathVec)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp       = rclcpp::Clock().now();
        pose.header.frame_id    = "map";
        pose.pose.position.x    = (p.first - this->startX) * gridSize;
        pose.pose.position.y    = (p.second - this->startY) * gridSize;
        pose.pose.position.z    = 0;
        pose.pose.orientation.x = 1;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 0;

        path.poses.push_back(pose);
    }

    return path;
}

void GridMap::expandGrid(const float &radius)
{
    expandedGrid.setZero();
    const int EXPAND_RADIUS = radius / gridSize;
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            setOnesAroundPoint(i, j, EXPAND_RADIUS);
        }
    }
    // cv::imshow("expandedGrid", expandedGridCV);
    RCLCPP_INFO(rclcpp::get_logger("GridMap::expandGrid"), "expanded");
}

void GridMap::expandGrid(const int &id, const float &radius)
{
    expandedGrid.setZero();
    const int EXPAND_RADIUS     = radius / gridSize;
    const int EXPAND_OBJ_RADIUS = 0.05 / gridSize;
    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            setOnesAroundPoint(i, j, EXPAND_RADIUS);
        }
    }

    for (const auto &stuff : stuffList)
    {
        if (stuff.first == id)
        {
            continue;
        }
        setOnesAroundPoint(
            stuff.second.first, stuff.second.second, EXPAND_OBJ_RADIUS);
    }

    // cv::imshow("expandedGrid", expandedGridCV);
    RCLCPP_INFO(rclcpp::get_logger("GridMap::expandGrid"), "expanded");
}

void GridMap::setOnesAroundPoint(const int &x, const int &y, const int &radius)
{
    const double EXPAND_THRESHOLD = 0.7;
    if (gridBeliefLiDAR(x, y) == 0.5 and gridBeliefRGBD(x, y) == 0.5)
    {
        expandedGrid(x, y) = 1;
        return;
    }

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

            // if (pow(i, 2) + pow(j, 2) <= pow(radius, 2) and
            //     (gridBeliefLiDAR(x, y) >= EXPAND_THRESHOLD or
            //      gridBeliefRGBD(x, y) >= EXPAND_THRESHOLD or
            //      knownGrid(x, y) == 1))
            // {
            //     expandedGrid(xOnGrid, yOnGrid) = 1;
            // }
            if (pow(i, 2) + pow(j, 2) <= pow(radius, 2) and
                (gridBeliefRGBD(x, y) >= EXPAND_THRESHOLD or
                 knownGrid(x, y) == 1))
            {
                expandedGrid(xOnGrid, yOnGrid) = 1;
            }
        }
    }
}

void GridMap::setLineSegmentOccupied(
    const std::vector<std::pair<double, double>> &lineSegments)
{
    for (size_t i = 0; i < lineSegments.size(); i++)
    {
        const double x1 = lineSegments[i].first;
        const double y1 = lineSegments[i].second;
        const double x2 = lineSegments[(i + 1) % lineSegments.size()].first;
        const double y2 = lineSegments[(i + 1) % lineSegments.size()].second;
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        const double d  = sqrt(dx * dx + dy * dy);
        const double nx = dx / d;
        const double ny = dy / d;
        for (double s = 0; s < d; s += gridSize)
        {
            knownGrid(cvFloor((x1 + s * nx) / gridSize) + startX,
                      cvFloor((y1 + s * ny) / gridSize) + startY) = 1;
        }
    }
}
void GridMap::updateStuffList(
    const std::map<int, std::pair<double, double>> &stuffList)
{
    this->stuffList.clear();
    for (const auto &stuff : stuffList)
    {
        int xOnGrid = cvFloor(stuff.second.first / gridSize) + startX;
        int yOnGrid = cvFloor(stuff.second.second / gridSize) + startY;

        this->stuffList[stuff.first] = std::make_pair(xOnGrid, yOnGrid);
    }
}

auto GridMap::getFrontier() -> std::vector<std::pair<int, int>>
{
    // get the frontier for frontier-based exploration
    std::vector<std::pair<int, int>> frontier;

    for (int i = 0; i < sizeX; i++)
    {
        for (int j = 0; j < sizeY; j++)
        {
            if (gridBeliefRGBD(i, j) == 0.5)  // unknown
            {
                bool isFrontier = false;
                for (int k = -1; k <= 1; k++)
                {
                    for (int l = -1; l <= 1; l++)
                    {
                        int x = i + k;
                        int y = j + l;
                        if (x < 0 or x >= sizeX or y < 0 or y >= sizeY)
                        {
                            continue;
                        }
                        if (expandedGrid(x, y) == 1)
                        {
                            isFrontier = true;
                            break;
                        }
                    }
                    if (isFrontier)
                    {
                        break;
                    }
                }
                if (isFrontier)
                {
                    frontier.emplace_back(i, j);
                }
            }
        }
    }

    return frontier;
}