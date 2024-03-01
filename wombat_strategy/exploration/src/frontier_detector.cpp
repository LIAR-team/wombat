// Copyright 2021-2022 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "wombat_strategy/exploration/frontier_detector.hpp"

#include <algorithm>
#include <limits>
#include <queue>

#include "wombat_core/costmap/costmap_conversions.hpp"
#include "wombat_core/grid/coordinates.hpp"
#include "wombat_core/grid/neighbors.hpp"
#include "wombat_core/math/geometry_point.hpp"

namespace wombat_strategy
{

FrontierDetector::FrontierDetector(const FrontierDetector::params_t & params)
: m_params(params)
{
  if (m_params.frontier_size_scaling_factor < 0.0f || m_params.frontier_size_scaling_factor > 1.0f) {
    const std::string factor_string = std::to_string(m_params.frontier_size_scaling_factor);
    throw std::runtime_error("Frontier scaling factor must be [0, 1], got: " + factor_string);
  }
}

std::vector<frontier_t> FrontierDetector::search_frontiers(
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
  const geometry_msgs::msg::Point & robot_position)
{
  if (!grid) {
    throw std::runtime_error("Invalid grid passed to search frontiers");
  }

  // Initialize vectors of booleans with size equal to the map size
  // to keep track of indices (i.e. map cells) already examined or already added to a frontier
  wombat_core::MapMetaDataAdapter map_info(grid->info);

  wombat_core::grid_index_t map_size = map_info.num_grid_cells();
  std::vector<bool> touched_indices(map_size, false);
  std::vector<bool> all_frontier_indices(map_size, false);

  auto maybe_starting_idx = wombat_core::world_pt_to_grid_index(robot_position, map_info);
  if (!maybe_starting_idx) {
    throw std::runtime_error("Failed to convert starting position into grid index");
  }

  // Create queue of indices to be visited, initialized with starting index
  std::queue<wombat_core::grid_index_t> to_be_visited;
  to_be_visited.push(*maybe_starting_idx);
  touched_indices[*maybe_starting_idx] = true;

  // Output data structure
  std::vector<frontier_t> frontiers;
  while (!to_be_visited.empty()) {
    // Pop front element out of the queue
    auto cell_idx = to_be_visited.front();
    to_be_visited.pop();
    // Check if a new frontier can be started from current cell
    if (is_frontier_cell(cell_idx, grid, map_info) && !all_frontier_indices[cell_idx]) {
      frontier_t f = build_frontier(cell_idx, grid, map_info, all_frontier_indices);
      if (frontier_is_valid(f, grid, map_info)) {
        frontiers.push_back(f);
      }
    }

    // Add neighbor cells to the queue
    wombat_core::for_each_grid_neighbor(
      cell_idx,
      map_info.grid_size.x(),
      map_info.grid_size.y(),
      [this, &touched_indices, &all_frontier_indices, &to_be_visited, &grid](wombat_core::grid_index_t i)
      {
        // Skip already visited cells
        if (touched_indices[i] || all_frontier_indices[i]) {
          return false;
        }
        // Mark this as touched to avoid looking at it again
        touched_indices[i] = true;
        // Add to queue if it's worth searching frontiers from this cell
        if (!m_params.search_only_free_space || grid->data[i] == wombat_core::occupancy::FREE) {
          to_be_visited.push(i);
        }
        // Keep going to next neighbor
        return false;
      },
      false);
  }

  rank_frontiers(robot_position, frontiers);
  return frontiers;
}

frontier_t FrontierDetector::build_frontier(
  wombat_core::grid_index_t starting_cell_idx,
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
  const wombat_core::MapMetaDataAdapter & map_info,
  std::vector<bool> & all_frontier_indices)
{
  // Output frontier
  frontier_t f;

  // We consider the first frontier cell found as the closest to the robot
  const auto maybe_start_pt = wombat_core::grid_index_to_world_pt(starting_cell_idx, map_info);
  if (!maybe_start_pt) {
    return f;
  }
  f.closest_point = *maybe_start_pt;

  // Queue of contiguous frontier cells
  std::queue<wombat_core::grid_index_t> to_be_visited;
  to_be_visited.push(starting_cell_idx);
  all_frontier_indices[starting_cell_idx] = true;

  while (!to_be_visited.empty()) {
    // Pop front element out of the queue
    wombat_core::grid_index_t cell_idx = to_be_visited.front();
    to_be_visited.pop();

    const auto maybe_frontier_pt = wombat_core::grid_index_to_world_pt(cell_idx, map_info);
    if (!maybe_frontier_pt) {
      throw std::runtime_error("Bad cell while building frontier");
    }
    f.points.push_back(*maybe_frontier_pt);

    wombat_core::for_each_grid_neighbor(
      cell_idx,
      map_info.grid_size.x(),
      map_info.grid_size.y(),
      [this, &all_frontier_indices, &to_be_visited, &grid, &map_info](wombat_core::grid_index_t i)
      {
        if (!all_frontier_indices[i] && is_frontier_cell(i, grid, map_info)) {
          to_be_visited.push(i);
          all_frontier_indices[i] = true;
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

void FrontierDetector::rank_frontiers(
  const geometry_msgs::msg::Point & robot_position,
  std::vector<frontier_t> & frontiers) const
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
    double distance = wombat_core::points_distance_2d(f.closest_point, robot_position);
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

bool FrontierDetector::frontier_is_valid(
  const frontier_t & f,
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
  const wombat_core::MapMetaDataAdapter & map_info,
  bool trusted)
{
  // Get number of cells that the frontier is made of
  size_t num_frontier_cells = 0;
  if (trusted) {
    num_frontier_cells = f.points.size();
  } else {
    for (const auto & point : f.points) {
      const auto maybe_cell_idx = wombat_core::world_pt_to_grid_index(point, map_info);
      if (!maybe_cell_idx) {
        return false;
      }
      if (is_frontier_cell(*maybe_cell_idx, grid, map_info)) {
        num_frontier_cells++;
      }
    }
  }

  // A frontier is invalid if its total number of cells is less than minimum required size
  return num_frontier_cells >= m_params.min_frontier_size;
}

bool FrontierDetector::is_frontier_cell(
  wombat_core::grid_index_t cell_idx,
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
  const wombat_core::MapMetaDataAdapter & map_info) const
{
  // A cell must be FREE in order to be a frontier
  if (grid->data[cell_idx] != wombat_core::occupancy::FREE) {
    return false;
  }

  // Examine neighbors of this cell
  uint8_t unknown_neighbors = 0;
  uint8_t free_neighbors = 0;
  uint8_t occupied_neighbors = 0;
  wombat_core::for_each_grid_neighbor(
    cell_idx,
    map_info.grid_size.x(),
    map_info.grid_size.y(),
    [&unknown_neighbors, &free_neighbors, &occupied_neighbors, &grid](wombat_core::grid_index_t i)
    {
      int8_t neighbor_cell_type = grid->data[i];
      if (neighbor_cell_type == wombat_core::occupancy::UNKNOWN) {
        unknown_neighbors++;
      } else if (neighbor_cell_type == wombat_core::occupancy::FREE) {
        free_neighbors++;
      } else if (neighbor_cell_type == wombat_core::occupancy::LETHAL_OBS) {
        occupied_neighbors++;
      }
      return false;
    },
    true);

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

}  // namespace wombat_strategy
