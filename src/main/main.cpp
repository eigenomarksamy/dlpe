/**
 * @file main.cpp
 * @author osamy
 * @brief int main() {}
 */

#include <iostream>
#include <random>
#include <astar.hpp>

/**
 * @brief execute the A* algorithm
 * @details 1) create object for algorithm
 *          2) run algorithm
 *          3) print the final grid using the pathVec
 * @param startNode - start node
 * @param goalNode - goal node
 * @param grid - grid to work with
 * @return void
 */
static void execAStar(Node_C& startNode, Node_C& goalNode, std::vector<std::vector<int64_t>>& grid);

static void execAStar(Node_C& startNode, Node_C& goalNode, std::vector<std::vector<int64_t>>& grid)
{
    std::cout << "algorithm: a*\n";
    planning::AStar_C aStar(grid);
    {
        const auto [pathFound, pathVec] = aStar.plan(startNode, goalNode);
        printPath(pathVec, startNode, goalNode, grid);
    }
}

#ifndef STANDALONE_BUILD
int main() {

    constexpr int64_t n = 21;
    std::vector<std::vector<int64_t>> grid(n, std::vector<int64_t>(n, 0));
    makeGrid(grid);

    /* obtain a random number from hardware */
    std::random_device rd;
    /* seed the generator */
    std::mt19937 eng(rd());
    /* define the range */
    std::uniform_int_distribution<int> distr(0, n - 1);

    Node_C start(distr(eng), distr(eng), 0, 0, 0, 0);
    Node_C goal(distr(eng), distr(eng), 0, 0, 0, 0);

    start.id_ = start.x_ * n + start.y_;
    start.pId_ = start.x_ * n + start.y_;
    goal.id_ = goal.x_ * n + goal.y_;
    start.hCost_ = abs(start.x_ - goal.x_) + abs(start.y_ - goal.y_);
    /* make sure start and goal are not obstacles and their ids are correctly assigned */
    grid[start.x_][start.y_] = 0;
    grid[goal.x_][goal.y_] = 0;
    printGrid(grid);

    /* store point after algorithm's run */
    std::vector<std::vector<int64_t>> mainGrid = grid;

    /* reset grid */
    grid = mainGrid;
    /* execute algorithm */
    execAStar(start, goal, grid);

    return 0;
}
#endif /* STANDALONE_BUILD */