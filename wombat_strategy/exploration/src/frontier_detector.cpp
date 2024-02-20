// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_strategy/exploration/frontier_detector.hpp"

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>

#include "wombat_core/costmap/costmap_utils.hpp"
#include "wombat_core/math/geometry_point.hpp"
#include "wombat_core/grid/neighbors.hpp"

namespace wombat_strategy
{

FrontierDetector::FrontierDetector(const FrontierDetector::params_t & params)
: m_params(params)
{
  bool valid_scale_factor =
    m_params.frontier_size_scaling_factor >= 0.0f && m_params.frontier_size_scaling_factor <= 1.0f;
  assert(valid_scale_factor && "Invalid scaling factor, must be [0, 1]");
}

std::vector<frontier_t> FrontierDetector::search_frontiers()
{
  // Output data structure
  std::vector<frontier_t> frontiers;

  assert(m_costmap != nullptr && "Costamp not initialized yet");

  // Initialize vectors of booleans with size equal to the map size
  // to keep track of indices (i.e. map cells) already examined or already added to a frontier
  unsigned int map_size = m_costmap->getSizeInCellsX() * m_costmap->getSizeInCellsY();
  std::vector<bool> touched_indices(map_size, false);
  std::vector<bool> already_included_frontier_indices(map_size, false);

  unsigned int starting_idx = wombat_core::world_to_index(m_robot_position, *m_costmap);
  // Create queue of indices to be visited, initialized with starting index
  std::queue<unsigned int> to_be_visited;
  to_be_visited.push(starting_idx);
  touched_indices[starting_idx] = true;

  while (!to_be_visited.empty()) {
    // Pop front element out of the queue
    unsigned int cell_idx = to_be_visited.front();
    to_be_visited.pop();
    // Check if a new frontier can be started from current cell
    if (is_frontier_cell(cell_idx) && !already_included_frontier_indices[cell_idx]) {
      frontier_t f = build_frontier(cell_idx, already_included_frontier_indices);
      if (frontier_is_valid(f)) {
        frontiers.push_back(f);
      }
    }

    // Add neighbor cells to the queue
    wombat_core::for_each_grid_neighbor(
      cell_idx,
      m_costmap->getSizeInCellsX(),
      m_costmap->getSizeInCellsY(),
      [this, &touched_indices, &already_included_frontier_indices, &to_be_visited](wombat_core::grid_index_t i)
      {
        // Skip already visited cells
        if (!touched_indices[i] && !already_included_frontier_indices[i]) {
          if (!m_params.search_only_free_space || m_costmap->getCharMap()[i] == FREE_CELL) {
            to_be_visited.push(i);
          }
        }
        return false;
      },
      false);
  }

  rank_frontiers(frontiers);
  return frontiers;
}

frontier_t FrontierDetector::build_frontier(
  unsigned int starting_cell_idx,
  std::vector<bool> & already_included_frontier_indices)
{
  // Output frontier
  frontier_t f;

  // We consider the first frontier cell found as the closest to the robot
  f.closest_point = wombat_core::index_to_world(starting_cell_idx, *m_costmap);

  // Queue of contiguous frontier cells
  std::queue<unsigned int> to_be_visited;
  to_be_visited.push(starting_cell_idx);
  already_included_frontier_indices[starting_cell_idx] = true;

  while (!to_be_visited.empty()) {
    // Pop front element out of the queue
    unsigned int cell_idx = to_be_visited.front();
    to_be_visited.pop();

    const auto frontier_point = wombat_core::index_to_world(cell_idx, *m_costmap);
    f.points.push_back(frontier_point);

    wombat_core::for_each_grid_neighbor(
      cell_idx,
      m_costmap->getSizeInCellsX(),
      m_costmap->getSizeInCellsY(),
      [this, &already_included_frontier_indices, &to_be_visited](wombat_core::grid_index_t i)
      {
        if (!already_included_frontier_indices[i] && is_frontier_cell(i)) {
          to_be_visited.push(i);
          already_included_frontier_indices[i] = true;
        }
        return false;
      },
      false);
  }

  // Compute centroid of this frontier
  geometry_msgs::msg::Point centroid;
  for (const auto & point : f.points) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / static_cast<double>(f.points.size());
  centroid.y = centroid.y / static_cast<double>(f.points.size());
  f.centroid = centroid;

  return f;
}

void FrontierDetector::rank_frontiers(std::vector<frontier_t> & frontiers)
{
  // Find minimum distance to a frontier and maximum frontier size to compute normalized scores
  double min_distance = std::numeric_limits<double>::max();
  size_t max_size = 0;
  // Vector to store distances, to avoid computing them twice
  std::vector<double> distances(frontiers.size());
  for (size_t i = 0; i < frontiers.size(); i++) {
    const frontier_t & f = frontiers[i];
    if (f.points.size() > max_size) {
      max_size = f.points.size();
    }
    double distance = wombat_core::points_distance_2d(f.closest_point, m_robot_position);
    distances[i] = distance;
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  // Compute and assign normalized score to each frontier object
  double distance_scaling_factor = 1.0f - m_params.frontier_size_scaling_factor;
  for (size_t i = 0; i < frontiers.size(); i++) {
    frontier_t & f = frontiers[i];
    double distance_score = distance_scaling_factor * min_distance / distances[i];
    double information_gain_score =
      m_params.frontier_size_scaling_factor * static_cast<double>(f.points.size()) / static_cast<double>(max_size);
    f.score = distance_score + information_gain_score;
  }

  // Sort frontiers according to the just assigned scores
  std::sort(frontiers.begin(), frontiers.end(), std::greater<frontier_t>());
}

bool FrontierDetector::frontier_is_valid(const frontier_t & f, bool trusted)
{
  // Get number of cells that the frontier is made of
  size_t num_frontier_cells = 0;
  if (trusted) {
    num_frontier_cells = f.points.size();
  } else {
    for (const auto & point : f.points) {
      const auto cell_idx = wombat_core::world_to_index(point, *m_costmap);
      if (is_frontier_cell(cell_idx)) {
        num_frontier_cells++;
      }
    }
  }

  // A frontier is invalid if its total number of cells is less than minimum required size
  return num_frontier_cells >= m_params.min_frontier_size;
}

}  // namespace wombat_strategy
