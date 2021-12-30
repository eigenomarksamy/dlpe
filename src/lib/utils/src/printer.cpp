/**
 * @file printer.cpp
 * @author osamy
 * @brief this is a file to be used to print data
 */

/* C/C++ standard includes */
#include <iostream>
#include <iomanip>

/* project-specific includes */
#include "utils.hpp"

// constants
constexpr int64_t spacing_for_grid = 10;


void printNodeStatus(const Node_C& node)
{
    std::cout << "-----------------" << '\n'
              << "Node_C          : " << '\n'
              << "x             : " << node.x_ << '\n'
              << "y             : " << node.y_ << '\n'
              << "Cost          : " << node.cost_ << '\n'
              << "Heuristic cost: " << node.hCost_ << '\n'
              << "Id            : " << node.id_ << '\n'
              << "Parent id     : " << node.pId_ << '\n'
              << "----------------" << '\n';
}

void printPath(const std::vector<Node_C>& pathVec, const Node_C& start,
               const Node_C& goal, std::vector<std::vector<int64_t>>& grid)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    if (pathVec.empty())
    {
        std::cout << "No path exists" << '\n';
        printGrid(grid);
        return;
    }
    std::cout << "Path (goal to start):" << '\n';
    for (size_t i = 0; i < pathVec.size(); i++)
    {
        if (compareCoordinates(goal, pathVec[i]))
        {
            pathVec[i].printStatus();
            grid[pathVec[i].x_][pathVec[i].y_] = 3;
            while (pathVec[i].id_ != start.id_)
            {
                if (pathVec[i].id_ == pathVec[i].pId_)
                {
                    break;
                }
                for (size_t j = 0; j < pathVec.size(); j++)
                {
                    if (pathVec[i].pId_ == pathVec[j].id_)
                    {
                        i = j;
                        pathVec[j].printStatus();
                        grid[pathVec[j].x_][pathVec[j].y_] = 3;
                    }
                }
            }
            break;
        }
    }
    grid[goal.x_][goal.y_] = 5;
    grid[start.x_][start.y_] = 4;
    printGrid(grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

void printCost(const std::vector<std::vector<int64_t>>& grid,
               const std::vector<Node_C>& pointVec)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    int64_t n = grid.size();
    std::vector<Node_C>::const_iterator it_v;
    for (int64_t i = 0; i < n; i++)
    {
        for (int64_t j = 0; j < n; j++)
        {
            for (it_v = pointVec.begin(); it_v != pointVec.end(); ++it_v)
            {
                if (i == it_v->x_ && j == it_v->y_)
                {
                    std::cout << std::setw(spacing_for_grid) << it_v->cost_ << " , ";
                    break;
                }
            }
            if (it_v == pointVec.end())
            {
                std::cout << std::setw(spacing_for_grid) << "  , ";
            }
        }
        std::cout << '\n' << '\n';
    }
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

void printPathInOrder(const std::vector<Node_C>& pathVec,
                      const Node_C& start, const Node_C& goal,
                      std::vector<std::vector<int64_t>>& grid)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    if (pathVec.empty())
    {
        std::cout << "Path not found" << '\n';
        printGrid(grid);
        return;
    }
    std::cout << "Path (goal to start):" << '\n';
    size_t i = 0;
    while (!compareCoordinates(pathVec[i], goal))
    {
        i++;
    }
    for (; i > 0; i = i - 1)
    {
        pathVec[i].printStatus();
        grid[pathVec[i].x_][pathVec[i].y_] = 3;
    }
    pathVec[0].printStatus();
    grid[pathVec[0].x_][pathVec[0].y_] = 3;
    printGrid(grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}