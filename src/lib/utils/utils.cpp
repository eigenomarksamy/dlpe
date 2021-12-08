/**
 * @file utils.cpp
 * @author osamy
 * @brief common functionalities and classes
 * for references see https://github.com/vss2sn/path_planning/blob/master/lib/utils/include/utils/utils.cpp
 */


/**
 * <TODO: remove iostream include and use files logging>
 */
#include <iostream>
#include <iomanip>
#include <random>
#include <stdint.h>

#include "utils.hpp"

// constants
constexpr uint64_t spacing_for_grid = 10;

void Node_C::printStatus() const
{
    std::cout << "-----------------" << '\n'
              << "Node_C          : " << '\n'
              << "x             : " << x_ << '\n'
              << "y             : " << y_ << '\n'
              << "Cost          : " << cost_ << '\n'
              << "Heuristic cost: " << hCost_ << '\n'
              << "Id            : " << id_ << '\n'
              << "Parent id     : " << pId_ << '\n'
              << "----------------" << '\n';
}

Node_C Node_C::operator+(const Node_C& p) const {
    Node_C tmp;
    tmp.x_ = this->x_ + p.x_;
    tmp.y_ = this->y_ + p.y_;
    tmp.cost_ = this->cost_ + p.cost_;
    return tmp;
}

Node_C Node_C::operator-(const Node_C& p) const {
  Node_C tmp;
  tmp.x_ = this->x_ - p.x_;
  tmp.y_ = this->y_ - p.y_;
  return tmp;
}

bool Node_C::operator==(const Node_C& p) const {
    return this->x_ == p.x_ && this->y_ == p.y_;
}

bool compareCoordinates(const Node_C& p1, const Node_C& p2)
{
    return p1.x_ == p2.x_ && p1.y_ == p2.y_;
}
bool checkOutsideBoundary(const Node_C& node, const uint64_t n)
{
    return (node.x_ < 0 || node.y_ < 0
        || node.x_ >= n || node.y_ >= n);
}

bool compare_cost_S::operator()(const Node_C& p1, const Node_C& p2) const {
    // Can modify this to allow tie breaks based on heuristic cost if required
    return p1.cost_ + p1.hCost_ > p2.cost_ + p2.hCost_ ||
            (p1.cost_ + p1.hCost_ == p2.cost_ + p2.hCost_ &&
            p1.hCost_ >= p2.hCost_);
}
bool compare_coord_S::operator()(const Node_C& p1, const Node_C& p2)  const {
    return p1.x_ == p2.x_ && p1.y_ == p2.y_;
}

// Possible motions for dijkstra, A*, and similar algorithms.
// Not using this for RRT & RRT* to allow random direction movements.
// TODO(vss): Consider adding option for motion restriction in RRT and RRT* by
//       replacing new node with nearest node that satisfies motion constraints
std::vector<Node_C> getPermissibleMotion()
{
    return {
        Node_C(0, 1, 1, 0, 0, 0),
        Node_C(1, 0, 1, 0, 0, 0),
        Node_C(0, -1, 1, 0, 0, 0),
        Node_C(-1, 0, 1, 0, 0, 0)
        // Node_C(1, 1, sqrt(2), 0, 0, 0),
        // Node_C(1, -1, sqrt(2), 0, 0, 0),
        // Node_C(-1, 1, sqrt(2), 0, 0, 0),
        // Node_C(-1, -1, sqrt(2), 0, 0, 0)
    };
    // NOTE: Add diagonal movements for A* and D* only after the heuristics in the
    // algorithms have been modified. Refer to README.md. The heuristics currently
    // implemented are based on Manhattan distance and dwill not account for
    // diagonal/ any other motions
}

void makeGrid(std::vector<std::vector<uint64_t>>& grid)
{
    uint64_t n = grid.size();
    std::random_device rd;   // obtain a random number from hardware
    std::mt19937 eng(rd());  // seed the generator
    std::uniform_int_distribution<uint64_t> distr(0, n);  // define the range

    for (uint64_t i = 0; i < n; i++)
    {
        for (uint64_t j = 0; j < n; j++)
        {
            grid[i][j] = distr(eng) / ((n - 1));  // probability of obstacle is 1/n
            // grid[i][j] = 0; // For no obstacles
        }
    }
}

void PrintPath(const std::vector<Node_C>& pathVec, const Node_C& start,
               const Node_C& goal, std::vector<std::vector<uint64_t>>& grid)
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
    grid[start.x_][start.y_] = 3;
    printGrid(grid);
#endif  // CUSTOM_DEBUG_HELPER_FUNCION
}

void printCost(const std::vector<std::vector<uint64_t>>& grid,
               const std::vector<Node_C>& pointVec)
{
#ifdef CUSTOM_DEBUG_HELPER_FUNCION
    uint64_t n = grid.size();
    std::vector<Node_C>::const_iterator it_v;
    for (uint64_t i = 0; i < n; i++)
    {
        for (uint64_t j = 0; j < n; j++)
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
                      std::vector<std::vector<uint64_t>>& grid)
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


void PrioQ_C::clear()
{
    s.clear();
    while(!pq.empty())
    {
        pq.pop();
    }
}

void PrioQ_C::insert(const node_key_pair_S& t)
{
    if (auto p = s.insert(t); !p.second)
    {
        s.erase(t);
        s.insert(t);
    }
    pq.push(t);
}

void PrioQ_C::pop()
{
    while(!pq.empty())
    {
        if (const auto it = s.find(pq.top()); it == s.end() ||
            (it != s.end() && pq.top().key != it->key))
        {
            // Element been removed from set OR
            // Element has been updated in set with new key, and inserted already into pq with new value
            pq.pop();
        }
        else if (it != s.end() && pq.top().key == it->key)
        {
            // Found an elelment that is in set and priority queue
            break;
        }
    }
    if (s.empty())
    {
        return;
    }
    s.erase(pq.top());
    pq.pop();
    // The loop below allows top() to be const without making the
    // std::priority_queue mutable
    while(!pq.empty())
    {
        if (const auto it = s.find(pq.top()); it == s.end() ||
            (it != s.end() && pq.top().key != it->key))
        {
            // Element been removed from set OR
            // Element has been updated in set with new key, and inserted already into pq with new value
            pq.pop();
        }
        else if (it != s.end() && pq.top().key == it->key)
        {
            // Found an elelment that is in set and priority queue
            break;
        }
    }
}

const node_key_pair_S& PrioQ_C::top() const
{
    return pq.top();
}

size_t PrioQ_C::size() const
{
    return s.size();
}

bool PrioQ_C::empty() const
{
    return s.empty();
}

bool PrioQ_C::isElementInStruct(const node_key_pair_S& t) const
{
    return s.find(t) != s.end();
}

void PrioQ_C::remove(const node_key_pair_S& t)
{
    if (s.find(t) != s.end())
    {
        s.erase(t);
    }
    // Ensure top() is const
    while(!pq.empty())
    {
        if (const auto it = s.find(pq.top()); it == s.end() ||
            (it != s.end() && pq.top().key != it->key))
        {
            // Element been removed from set OR
            // Element has been updated in set with new key, and inserted already into pq with new value
            pq.pop();
        }
        else if (it != s.end() && pq.top().key == it->key)
        {
            // Found an elelment that is in set and priority queue
            break;
        }
    }
}