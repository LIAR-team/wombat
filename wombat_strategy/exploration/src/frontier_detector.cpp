// Copyright 2021 Azzollini Ilario, Gentilini Lorenzo, Soragna Alberto, Tazzari Roberto.
// All Rights Reserved.

#include "wombat_strategy/exploration/frontier_detector.hpp"

#include <assert.h>
#include <algorithm>
#include <iostream>
#include <limits>
#include <queue>

#include "wombat_strategy/exploration/utilities.hpp"

namespace wombat_strategy
{

FrontierDetector::FrontierDetector(const FrontierDetector::Params & params)
: m_params(params)
{
  bool valid_scale_factor =
    m_params.frontier_size_scaling_factor >= 0.0f && m_params.frontier_size_scaling_factor <= 1.0f;
  assert(valid_scale_factor && "Invalid scaling factor, must be [0, 1]");
}

std::vector<Frontier> FrontierDetector::search_frontiers()
{
  // Output data structure
  std::vector<Frontier> frontiers;

  assert(m_costmap != nullptr && "Costamp not initialized yet");

  // Initialize vectors of booleans with size equal to the map size
  // to keep track of indices (i.e. map cells) already examined or already added to a frontier
  size_t map_size = m_costmap->getSizeInCellsX() * m_costmap->getSizeInCellsY();
  std::vector<bool> touched_indices(map_size, false);
  std::vector<bool> already_included_frontier_indices(map_size, false);

  unsigned int starting_idx = world_to_index(m_robot_position, m_costmap);
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
      Frontier f = build_frontier(cell_idx, already_included_frontier_indices);
      if (frontier_is_valid(f)) {
        frontiers.push_back(f);
      }
    }

    // Add neighbor cells to the queue
    for (const auto neighbor_cell_idx : get_neighbors4(cell_idx)) {
      // Skip already visited cells
      if (touched_indices[neighbor_cell_idx] || already_included_frontier_indices[neighbor_cell_idx]) {
        continue;
      }
      if (!m_params.search_only_free_space || m_costmap->getCharMap()[neighbor_cell_idx] == FREE_CELL) {
        to_be_visited.push(neighbor_cell_idx);
      }
      touched_indices[neighbor_cell_idx] = true;
    }
  }

  rank_frontiers(frontiers);
  return frontiers;
}

Frontier FrontierDetector::build_frontier(
  unsigned int starting_cell_idx,
  std::vector<bool> & already_included_frontier_indices)
{
  // Output frontier
  Frontier f;

  // We consider the first frontier cell found as the closest to the robot
  f.closest_point = index_to_world(starting_cell_idx, m_costmap);

  // Queue of contiguous frontier cells
  std::queue<unsigned int> to_be_visited;
  to_be_visited.push(starting_cell_idx);
  already_included_frontier_indices[starting_cell_idx] = true;

  while (!to_be_visited.empty()) {
    // Pop front element out of the queue
    unsigned int cell_idx = to_be_visited.front();
    to_be_visited.pop();

    const auto frontier_point = index_to_world(cell_idx, m_costmap);
    f.points.push_back(frontier_point);

    for (const auto neighbor_cell_idx : get_neighbors4(cell_idx)) {
      if (!already_included_frontier_indices[neighbor_cell_idx] && is_frontier_cell(neighbor_cell_idx)) {
        to_be_visited.push(neighbor_cell_idx);
        already_included_frontier_indices[neighbor_cell_idx] = true;
      }
    }
  }

  // Compute centroid of this frontier
  geometry_msgs::msg::Point centroid;
  for (const auto & point : f.points) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / f.points.size();
  centroid.y = centroid.y / f.points.size();
  f.centroid = centroid;

  return f;
}

void FrontierDetector::rank_frontiers(std::vector<Frontier> & frontiers)
{
  // Find miminum distance to a frontier and maximum frontier size to compute normalized scores
  double min_distance = std::numeric_limits<double>::max();
  size_t max_size = 0;
  // Vector to store distances, to avoid computing them twice
  std::vector<double> distances(frontiers.size());
  for (size_t i = 0; i < frontiers.size(); i++) {
    const Frontier & f = frontiers[i];
    if (f.points.size() > max_size) {
      max_size = f.points.size();
    }
    double distance = points_distance(f.closest_point, m_robot_position);
    distances[i] = distance;
    if (distance < min_distance) {
      min_distance = distance;
    }
  }

  // Compute and assign normalized score to each frontier object
  double distance_scaling_factor = 1.0f - m_params.frontier_size_scaling_factor;
  for (size_t i = 0; i < frontiers.size(); i++) {
    Frontier & f = frontiers[i];
    double distance_score = distance_scaling_factor * min_distance / distances[i];
    double information_gain_score = m_params.frontier_size_scaling_factor * f.points.size() / max_size;
    f.score = distance_score + information_gain_score;
  }

  // Sort frontiers according to the just assigned scores
  std::sort(frontiers.begin(), frontiers.end(), std::greater<Frontier>());
}

bool FrontierDetector::frontier_is_valid(const Frontier & f, bool trusted)
{
  // Get number of cells that the frontier is made of
  size_t num_frontier_cells = 0;
  if (trusted) {
    num_frontier_cells = f.points.size();
  } else {
    for (const auto & point : f.points) {
      const auto cell_idx = world_to_index(point, m_costmap);
      if (is_frontier_cell(cell_idx)) {
        num_frontier_cells++;
      }
    }
  }

  // A frontier is invalid if its total number of cells is less than minimum required size
  return num_frontier_cells >= m_params.min_frontier_size;
}

}  // namespace wombat_strategy