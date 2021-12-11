/**
 * @file astar.cpp
 * @author osamy
 * @brief contains the A* class implementation
 */

#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

#ifdef STANDALONE_BUILD
#include <random>
#endif /* STANDALONE_BUILD */

#include "astar.hpp"

std::tuple<bool, std::vector<Node_C>> planning::AStar_C::plan(const Node_C& start,
                                                              const Node_C& goal)
{
    grid_ = original_grid_;
    std::priority_queue<Node_C, std::vector<Node_C>, compare_cost_S> oList;
    std::unordered_set<Node_C, NodeIdHash_C, compare_coord_S> cList;

    const std::vector<Node_C> perMotion = getPermissibleMotion();

    oList.push(start);

    while (!oList.empty())
    {
        Node_C cur = oList.top();
        oList.pop();

        cur.id_ = cur.x_ * n_ + cur.y_;

        if (compareCoordinates(cur, goal))
        {
            cList.insert(cur);
            grid_[cur.x_][cur.y_] = 2;
            return {true, convertClosedList2Path(cList, start, goal)};
        }

        grid_[cur.x_][cur.y_] = 2;

        for (const auto& pm : perMotion)
        {
            Node_C newPoint = cur + pm;
            newPoint.id_ = n_ * newPoint.x_ + newPoint.y_;
            newPoint.pId_ = cur.id_;
            newPoint.hCost_ = fabs(newPoint.x_ - goal.x_) + fabs(newPoint.y_ - goal.y_);

            if (compareCoordinates(newPoint, goal))
            {
                oList.push(newPoint);
                break;
            }
            if (checkOutsideBoundary(newPoint, n_))
            {
                continue;
            }
            if (0 != grid_[newPoint.x_][newPoint.y_])
            {
                continue;
            }
            oList.push(newPoint);
        }
        cList.insert(cur);
    }
    return {false, {}};
}

std::vector<Node_C> planning::AStar_C::convertClosedList2Path(
                                        std::unordered_set<Node_C, NodeIdHash_C, compare_coord_S>& cList,
                                        const Node_C& start, const Node_C& goal)
{
    auto cur = *cList.find(goal);
    std::vector<Node_C> path;

    while (!compareCoordinates(cur, start))
    {
        path.push_back(cur);

        if (const auto it = cList.find(Node_C(cur.pId_ / n_, cur.pId_ % n_, 0, 0, cur.pId_));
            it != cList.end())
        {
            cur = *it;
        }
        else
        {
            std::cout << "Error in calculating path\n";
            return {};
        }
    }
    path.push_back(start);
    return path;
}

#ifdef STANDALONE_BUILD
/**
 * @brief script main function. generates start and end nodes as well as grid,
 * then creates the algorithm object and calls the main algorithm function.
 * @return 0
 */
int main()
{
    constexpr int n = 11;
    std::vector<std::vector<uint64_t>> grid(n, std::vector<uint64_t>(n, 0));

    makeGrid(grid);

    std::random_device rd;
    std::mt19937 eng(rd());
    std::uniform_int_distribution<uint64_t> distr(0, n - 1);

    Node_C start(distr(eng), distr(eng), 0, 0, 0, 0);
    Node_C goal(distr(eng), distr(eng), 0, 0, 0, 0);

    start.id_ = start.x_ * n + start.y_;
    start.pId_ = start.id_;
    goal.id_ = goal.x_ * n + goal.y_;
    start.hCost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);

    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;

    start.printStatus();
    goal.printStatus();

    printGrid(grid);

    planning::AStar_C newAStar(grid);
    const auto [pathFound, pathVector] = newAStar.plan(start, goal);

    printPath(pathVector, start, goal, grid);

    return 0;
}
#endif /* STANDALONE_BUILD */