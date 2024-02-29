// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "wombat_core/grid/types.hpp"
#include "wombat_strategy/exploration/frontier.hpp"

namespace wombat_strategy
{

class FrontierDetector
{
public:
  /** @brief Parameters to customize the frontier detection operation */
  struct params_t
  {
    // If true frontiers search propagates only through FREE cells
    bool search_only_free_space {false};
    // A cell is a frontier if it has at least this number of neighbors unknown cells
    size_t min_unknown_neighbors {1};
    // A cell is a frontier if it has at least this number of neighbors free cells
    size_t min_free_neighbors {1};
    // A cell is a frontier if it has less than this number of neighbors occupied cells
    size_t max_occupied_neighbors {8};
    // Frontiers made of less than this number of cells are discarded
    size_t min_frontier_size {0};
    // [0, 1] scaling factor for ranking frontiers
    double frontier_size_scaling_factor {0.0};
  };

  /** @brief Default constructor */
  FrontierDetector() = default;

  /**
   * @brief Class constructor to store provided parameters
   * @param params configuration parameters
   */
  explicit FrontierDetector(const params_t & params);

  /**
    * @brief Main API, returns a ranked list of frontiers computed on the current occupancy grid
    * @param grid occupancy grid
    * @param robot_position current robot position
    * @return frontiers ranked from best to worst
    */
  std::vector<frontier_t> search_frontiers(
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
    const geometry_msgs::msg::Point & robot_position);

  /**
    * @brief Helper function that checks if a frontier is valid.
    * @param frontier the frontier to check
    * @param grid occupancy grid
    * @param trusted whether the map has been updated or not since when the frontier was computed
    * @return true if the frontier is still valid
    */
  bool frontier_is_valid(
    const frontier_t & frontier,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
    const wombat_core::MapMetaDataAdapter & map_info,
    bool trusted = true);

private:
  /**
    * @brief Create a frontier object starting from a given index.
    * While the frontier is created, its indices are also added to the all_frontier_indices
    * NOTE: This method will not assign a score to the frontier
    * @param starting_cell_idx first identified index belonging to the frontier
    * @param grid occupancy grid
    * @param all_frontier_indices
    * @return frontier_t the constructed frontier
    */
  frontier_t build_frontier(
    wombat_core::grid_index_t starting_cell_idx,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
    const wombat_core::MapMetaDataAdapter & map_info,
    std::vector<bool> & all_frontier_indices);

  /**
    * @brief Assign a score and ranks the frontiers according to it
    * @param frontiers will be re-ordered according to their descending score
    */
  void rank_frontiers(
    const geometry_msgs::msg::Point & robot_position,
    std::vector<frontier_t> & frontiers);

  /**
    * @brief checks if a cell denotes a frontier on the current costmap
    * @param cell_idx to be checked
    * @return true if the index denotes a frontier cell
    */
  bool is_frontier_cell(
    wombat_core::grid_index_t cell_idx,
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
    const wombat_core::MapMetaDataAdapter & map_info);

  params_t m_params;
};

}  // namespace wombat_strategy
