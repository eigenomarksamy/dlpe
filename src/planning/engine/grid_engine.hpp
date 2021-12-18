/**
 * @file grid_engine.hpp
 * @author osamy
 * @brief grid planning engine abstract class
 */

#ifndef GRID_ENGINE_H_
#define GRID_ENGINE_H_

/**
 * <TODO: remove iostream include and use files logging>
 */
#include <iostream>
#include <stdint.h>
#include <vector>
#include <tuple>
#include <unordered_map>

#include "utils.hpp"

namespace planning
{

/**
 *  abstract class that is inherited by concerete implementaions of grid planner
 *  classes. the plan function is a pure virtual funciton that is overloaded
 *  <TODO: wrap types and log into out files>
 */
class GPEngine_C
{

public:
    /**
     * @brief constructor
     * @param grid the grid on which the planner is to plan
     * @return no return value
     */
    GPEngine_C(std::vector<std::vector<int64_t>> grid)
      : original_grid_(std::move(grid)), n_(original_grid_.size()){};

    /**
     * @brief copy constructor
     * @return no return
     * @details default
     */
    GPEngine_C(const GPEngine_C&) = default;

    /**
     * @brief copy constructor
     * @return no return value
     * @details defautl
     */
    GPEngine_C(GPEngine_C&&) = default;

    /**
     * @brief copy assignment
     * @return no return value
     * @details operator overloading
     */
    GPEngine_C& operator=(const GPEngine_C&) = default;

    /**
     * @brief move assignment
     * @return no return value
     * @details operator overloading
     */
    GPEngine_C& operator=(GPEngine_C&&) = default;

    /**
     * @brief virtual destructor
     * @return no return value
     * @details destructor
     */
    virtual ~GPEngine_C() = default;

    /**
     * @brief pure virtual function, overloaded by each of planners' implementations
     * @param start - start node
     * @param goal - goal node
     * @return tuple containing bool, if there is a path, path
     */
    virtual std::tuple<bool, std::vector<Node_C>> plan(const Node_C& start, const Node_C& goal) = 0;

    /**
     * @brief sets the time discovered obstacles and flag to create random ones
    * @param createRandObst - should random obstacles be created during execution
    * @param timeDiscObst - obstacles to be discovered at specific times
    * @return void
    * @details set separately from the plan function to allow this to persist between calls to plan()
    */
    virtual void setDynamicObstacles(const bool createRandObst = false,
                                     const std::unordered_map<int64_t, std::vector<Node_C>>& timeDiscObst = {})
    {
        std::cout << "Please implement this function for the planner" << '\n';
        std::cout << "Value attempted to be set: " << '\n';
        std::cout << "Create random obstacles: " << createRandObst << '\n';
        std::cout << "Number of time discovered obstacles: " << timeDiscObst.size() << '\n';
    };

protected:
    std::vector<std::vector<int64_t>> grid_ = {};
    const std::vector<std::vector<int64_t>> original_grid_;
    const int64_t n_;
};

} // namespace planning

#endif /* GRID_ENGINE_H_ */