// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "wombat_strategy/exploration/frontier.hpp"

namespace wombat_strategy
{

class FrontierDetector
{
public:
  /** @brief Parameters to customize the frontier detection operation */
  struct Params
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

  /** @brief Class constructor, simply stores provided parameters */
  explicit FrontierDetector(const Params & params);

  /**
    * @brief Provide a pointer to a costmap that is used for frontier detection.
    * @param costmap that is meant to be updated from the application
    */
  void set_costmap(std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap)
  {
    m_costmap = std::move(costmap);
  }

  /**
    * @brief Set current robot position, used for frontier detection and ranking.
    * @param robot_position in world reference frame
    */
  void set_current_position(geometry_msgs::msg::Point robot_position)
  {
    m_robot_position = robot_position;
  }

  /**
    * @brief Main API, returns a ranked list of frontiers computed on the current costmap.
    * It requires that a valid pointer to a costmap has already been provided.
    * @return frontiers ranked from best to worst
    */
  std::vector<Frontier> search_frontiers();

  /**
    * @brief Helper function that checks if a frontier is valid.
    * @param frontier the frontier to check
    * @param trusted whether the map has been updated or not since when the frontier was computed
    * @return true if the frontier is still valid
    */
  bool frontier_is_valid(const Frontier & frontier, bool trusted = true);

private:
  static constexpr unsigned int FREE_CELL = 0;
  static constexpr unsigned int OCCUPIED_CELL = 100;
  static constexpr unsigned int UNKNOWN_CELL = 255;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> m_costmap;
  geometry_msgs::msg::Point m_robot_position;  // robot position in world frame
  Params m_params;

  /**
    * @brief Create a frontier object starting from a given index.
    * While the frontier is created, its indices are also added to the already_included_frontier_indices
    * NOTE: This method will not assign a score to the frontier
    * @param starting_cell_idx first identified index belonging to the frontier
    * @param already_included_frontier_indices
    */
  Frontier build_frontier(unsigned int starting_cell_idx, std::vector<bool> & already_included_frontier_indices);

  /**
    * @brief Assign a score and ranks the frontiers according to it
    * @param frontiers will be re-ordered according to their descending score
    */
  void rank_frontiers(std::vector<Frontier> & frontiers);

  /**
    * @brief checks if a cell denotes a frontier on the current costmap
    * @param cell_idx to be checked
    * @return true if the index denotes a frontier cell
    */
  bool is_frontier_cell(unsigned int cell_idx)
  {
    // A cell must be FREE in order to be a frontier
    if (m_costmap->getCharMap()[cell_idx] != FREE_CELL) {
      return false;
    }

    // Examine neighbors of this cell
    uint8_t unknown_neighbors = 0;
    uint8_t free_neighbors = 0;
    uint8_t occupied_neighbors = 0;
    for (auto neighbor_cell_idx : get_neighbors8(cell_idx)) {
      unsigned int neighbor_cell_type = m_costmap->getCharMap()[neighbor_cell_idx];
      if (neighbor_cell_type == UNKNOWN_CELL) {
        unknown_neighbors++;
      } else if (neighbor_cell_type == FREE_CELL) {
        free_neighbors++;
      } else if (neighbor_cell_type == OCCUPIED_CELL) {
        occupied_neighbors++;
      }
    }

    // Discard cells which do not meet the requirements
    if (unknown_neighbors < m_params.min_unknown_neighbors) {
      return false;
    }
    if (free_neighbors < m_params.min_free_neighbors) {
      return false;
    }
    if (occupied_neighbors >= m_params.max_occupied_neighbors) {
      return false;
    }

    return true;
  }

  /**
    * @brief Get N-S-W-E neighbor indices from the given index, checking for out of bounds
    * @param cell_idx from which neighbors are searched
    * @return vector of up to 4 neighbors
    */
  std::vector<unsigned int> get_neighbors4(unsigned int cell_idx)
  {
    std::vector<unsigned int> neighbors;

    unsigned int width = m_costmap->getSizeInCellsX();
    unsigned int height = m_costmap->getSizeInCellsY();

    // The cell index is outside the map, no neighbors
    if (cell_idx > width * height - 1) {
      return neighbors;
    }

    if (cell_idx % width > 0) {
      neighbors.push_back(cell_idx - 1);
    }
    if (cell_idx % width < width - 1) {
      neighbors.push_back(cell_idx + 1);
    }
    if (cell_idx >= width) {
      neighbors.push_back(cell_idx - width);
    }
    if (cell_idx < width * (height - 1)) {
      neighbors.push_back(cell_idx + width);
    }

    return neighbors;
  }

  /**
    * @brief Get N-NE-NW-S-SE-SW-W-E neighbor indices from the given index, checking for out of bounds
    * @param cell_idx from which neighbors are searched
    * @return vector of up to 8 neighbors
    */
  inline std::vector<unsigned int> get_neighbors8(unsigned int cell_idx)
  {
    std::vector<unsigned int> neighbors;

    unsigned int width = m_costmap->getSizeInCellsX();
    unsigned int height = m_costmap->getSizeInCellsY();

    // The cell index is outside the map, no neighbors
    if (cell_idx > width * height - 1) {
      return neighbors;
    }

    neighbors = get_neighbors4(cell_idx);

    if (cell_idx % width > 0 && cell_idx >= width) {
      neighbors.push_back(cell_idx - 1 - width);
    }
    if (cell_idx % width > 0 && cell_idx < width * (height - 1)) {
      neighbors.push_back(cell_idx - 1 + width);
    }
    if (cell_idx % width < width - 1 && cell_idx >= width) {
      neighbors.push_back(cell_idx + 1 - width);
    }
    if (cell_idx % width < width - 1 && cell_idx < width * (height - 1)) {
      neighbors.push_back(cell_idx + 1 + width);
    }

    return neighbors;
  }
};

}  // namespace wombat_strategy
