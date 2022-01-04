/**
 * @file utils.cpp
 * @author osamy
 * @brief common functionalities and classes
 * for references see https://github.com/vss2sn/path_planning/blob/master/lib/utils/include/utils/utils.cpp
 */

#include <random>
#include <stdint.h>

#include "utils.hpp"

void Node_C::printStatus() const
{
    printNodeStatus(*this);
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
bool checkOutsideBoundary(const Node_C& node, const int64_t n)
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

void makeGrid(std::vector<std::vector<int64_t>>& grid)
{
    int64_t n = grid.size();
    std::random_device rd;   // obtain a random number from hardware
    std::mt19937 eng(rd());  // seed the generator
    std::uniform_int_distribution<int64_t> distr(0, n);  // define the range

    for (int64_t i = 0; i < n; i++)
    {
        for (int64_t j = 0; j < n; j++)
        {
            grid[i][j] = distr(eng) / ((n - 1));  // probability of obstacle is 1/n
            // grid[i][j] = 0; // For no obstacles
        }
    }
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