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

  auto maybe_starting_idx = wombat_core::world_pt_to_grid_index(robot_position, map_info);
  if (!maybe_starting_idx) {
    throw std::runtime_error("Failed to convert starting position into grid index");
  }

  // Create queue of indices to be visited, initialized with starting index
  std::unordered_map<wombat_core::grid_index_t, IndexType> index_type_storage;
  std::queue<wombat_core::grid_index_t> map_queue;
  map_queue.push(*maybe_starting_idx);
  index_type_storage[*maybe_starting_idx] = static_cast<IndexType>(index_type_storage[*maybe_starting_idx] | IndexType::MAP_OPEN);

  // Output data structure
  std::vector<frontier_t> frontiers;
  while (!map_queue.empty()) {
    // Pop front element out of the queue
    const auto grid_idx = map_queue.front();
    map_queue.pop();

    // Nothing to do for map closed elements
    if ((index_type_storage[grid_idx] & IndexType::MAP_CLOSED) != 0) {
      continue;
    }
    // Check if a new frontier can be started from current cell
    if (is_frontier_cell(grid_idx, grid, map_info)) {
      const auto frontier_indices = enumerate_frontier(
        grid_idx,
        grid,
        map_info,
        index_type_storage);
      if (frontier_indices.size() >= m_params.min_frontier_size) {
        const auto new_frontier = build_frontier(frontier_indices, grid_idx, map_info);
        frontiers.push_back(new_frontier);
      }
    }

    // Add neighbor cells to the queue
    wombat_core::for_each_grid_neighbor(
      grid_idx, map_info.grid_size.x(), map_info.grid_size.y(),
      [this, &index_type_storage, &map_queue, &grid](wombat_core::grid_index_t i)
      {
        if ((index_type_storage[i] & (IndexType::MAP_OPEN | IndexType::MAP_CLOSED)) != 0) {
          return false;
        }

        // Add to queue if it's worth searching frontiers from this cell
        if (!m_params.search_only_free_space || grid->data[i] == wombat_core::occupancy::FREE) {
          index_type_storage[i] = static_cast<IndexType>(index_type_storage[i] | IndexType::MAP_OPEN);
          map_queue.push(i);
        }
        // Keep going to next neighbor
        return false;
      },
      false);

    index_type_storage[grid_idx] = static_cast<IndexType>(index_type_storage[grid_idx] | IndexType::MAP_CLOSED);
  }

  rank_frontiers(robot_position, frontiers);
  return frontiers;
}

frontier_t FrontierDetector::build_frontier(
  const std::vector<wombat_core::grid_index_t> & frontier_indices,
  wombat_core::grid_index_t starting_cell_idx,
  const wombat_core::MapMetaDataAdapter & map_info)
{
  // Output frontier
  frontier_t f;
  // We consider the first frontier cell found as the closest to the robot
  const auto maybe_start_pt = wombat_core::grid_index_to_world_pt(starting_cell_idx, map_info);
  if (!maybe_start_pt) {
    throw std::runtime_error("Bad starting cell while building frontier");
  }
  f.closest_point = *maybe_start_pt;

  f.points.reserve(frontier_indices.size());
  geometry_msgs::msg::Point centroid;
  for (const auto & cell_idx : frontier_indices) {
    const auto maybe_frontier_pt = wombat_core::grid_index_to_world_pt(cell_idx, map_info);
    if (!maybe_frontier_pt) {
      throw std::runtime_error("Bad cell while building frontier");
    }
    f.points.push_back(*maybe_frontier_pt);

    centroid.x += maybe_frontier_pt->x;
    centroid.y += maybe_frontier_pt->y;
  }
  centroid.x = centroid.x / static_cast<double>(f.points.size());
  centroid.y = centroid.y / static_cast<double>(f.points.size());
  f.centroid = centroid;

  return f;
}

std::vector<wombat_core::grid_index_t>
FrontierDetector::enumerate_frontier(
  wombat_core::grid_index_t starting_cell_idx,
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr grid,
  const wombat_core::MapMetaDataAdapter & map_info,
  std::unordered_map<wombat_core::grid_index_t, IndexType> & index_type_storage)
{
  // Queue of contiguous frontier cells
  std::vector<wombat_core::grid_index_t> frontier_indices;
  std::queue<wombat_core::grid_index_t> to_be_visited;
  to_be_visited.push(starting_cell_idx);
  index_type_storage[starting_cell_idx] = static_cast<IndexType>(index_type_storage[starting_cell_idx] | IndexType::FRONTIER_OPEN);

  while (!to_be_visited.empty()) {
    // Pop front element out of the queue
    wombat_core::grid_index_t this_grid_idx = to_be_visited.front();
    to_be_visited.pop();
    // Nothing to do for map closed elements
    if ((index_type_storage[this_grid_idx] & (IndexType::MAP_CLOSED | IndexType::FRONTIER_CLOSED)) != 0) {
      continue;
    }
    index_type_storage[this_grid_idx] = static_cast<IndexType>(index_type_storage[this_grid_idx] | IndexType::FRONTIER_CLOSED);
    if (!is_frontier_cell(this_grid_idx, grid, map_info)) {
      continue;
    }

    frontier_indices.push_back(this_grid_idx);

    wombat_core::for_each_grid_neighbor(
      this_grid_idx,
      map_info.grid_size.x(),
      map_info.grid_size.y(),
      [this, &index_type_storage, &to_be_visited](wombat_core::grid_index_t i)
      {
        static constexpr unsigned int invalid_types
          {IndexType::MAP_CLOSED | IndexType::FRONTIER_OPEN | IndexType::FRONTIER_CLOSED};
        if ((index_type_storage[i] & invalid_types) == 0) {
          index_type_storage[i] = static_cast<IndexType>(index_type_storage[i] | IndexType::FRONTIER_OPEN);
          to_be_visited.push(i);
        }
        return false;
      },
      false);
  }

  for (const auto & this_grid_idx : frontier_indices) {
    index_type_storage[this_grid_idx] = static_cast<IndexType>(index_type_storage[this_grid_idx] | IndexType::FRONTIER_CLOSED);
  }

  return frontier_indices;
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
  const wombat_core::MapMetaDataAdapter & map_info)
{
  // Get number of cells that the frontier is made of
  size_t num_frontier_cells = 0;
  for (const auto & point : f.points) {
    const auto maybe_cell_idx = wombat_core::world_pt_to_grid_index(point, map_info);
    if (!maybe_cell_idx) {
      return false;
    }
    if (is_frontier_cell(*maybe_cell_idx, grid, map_info)) {
      num_frontier_cells++;
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
