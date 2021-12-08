/**
 * @file astar.hpp
 * @author osamy
 * @brief astar planner class
 */

#ifndef ASTAR_H_
#define ASTAR_H_

#include <queue>

#include "grid_engine.hpp"
#include "utils.hpp"

namespace planning
{

/**
 * @brief class for using A* algorithm
 */
class AStar_C : public GPEngine_C
{
public:
    /**
     * @brief constructor
     * @param grid - grid map for the planning task
     * @return none
     */
    explicit AStar_C(std::vector<std::vector<uint64_t>> grid)
                : GPEngine_C(std::move(grid)) {}

    /**
     * @brief algorithm's implementation
     * @param start - start node
     * @param goal - goal node
     * @return typle contains a bool to whether there was a path,
     * with the respective path.
     */
    std::tuple<bool, std::vector<Node_C>> plan(const Node_C& start,
                                               const Node_C& goal) override;

private:
    std::vector<Node_C> convertClosedList2Path(std::unordered_set<Node_C, NodeIdHash_C, compare_coord_S>& cList,
                                               const Node_C& start, const Node_C& goal);
};


} // namespace planning

#endif /* ASTAR_H_ */