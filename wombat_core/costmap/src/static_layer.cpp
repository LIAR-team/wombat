// Copyright 2024 Soragna Alberto.

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include <algorithm>
#include <memory>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2/time.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "wombat_core/costmap/costmap_conversions.hpp"
#include "wombat_core/costmap/static_layer.hpp"


namespace wombat_core
{

void StaticLayer::setup(nav2_costmap_2d::LayeredCostmap * parent)
{
  name_ = "kennel_static";
  layered_costmap_ = parent;

  m_global_frame = "ground_truth";
  enabled_ = true;
  m_track_unknown_space = false;
  m_use_maximum = false;
  m_lethal_threshold = 100;
  m_unknown_cost_value = static_cast<unsigned char>(0xff);
  m_trinary_costmap = true;
  m_transform_tolerance = tf2::durationFromSec(0.0);
}

void
StaticLayer::reset()
{
  m_has_updated_data = true;
  current_ = false;
}

void
StaticLayer::update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr new_map)
{
  if (!m_map_received) {
    processMap(*new_map);
    m_map_received = true;
    return;
  }
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  m_map_buffer = new_map;
}

void
StaticLayer::update_map(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (update->y < static_cast<int32_t>(m_y) ||
    m_y + m_height < update->y + update->height ||
    update->x < static_cast<int32_t>(m_x) ||
    m_x + m_width < update->x + update->width)
  {
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Exceeds bounds of static layer.\n"
      "Static layer origin: %d, %d   bounds: %d X %d\n"
      "Update origin: %d, %d   bounds: %d X %d",
      m_x, m_y, m_width, m_height, update->x, update->y, update->width,
      update->height);
    return;
  }

  if (update->header.frame_id != m_map_frame) {
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Current map is in frame %s "
      "but update was in frame %s",
      m_map_frame.c_str(), update->header.frame_id.c_str());
  }

  unsigned int di = 0;
  for (unsigned int y = 0; y < update->height; y++) {
    unsigned int index_base = (update->y + y) * size_x_;
    for (unsigned int x = 0; x < update->width; x++) {
      unsigned int index = index_base + x + update->x;
      costmap_[index] = interpretValue(update->data[di++]);
    }
  }

  m_has_updated_data = true;
}

void
StaticLayer::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  RCLCPP_DEBUG(logger_, "StaticLayer: Process map");

  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;

  RCLCPP_DEBUG(
    logger_,
    "StaticLayer: Received a %d X %d map at %f m/pix", size_x, size_y,
    new_map.info.resolution);

  // resize costmap if size, resolution or origin do not match
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (master->getSizeInCellsX() != size_x ||
    master->getSizeInCellsY() != size_y ||
    master->getResolution() != new_map.info.resolution ||
    master->getOriginX() != new_map.info.origin.position.x ||
    master->getOriginY() != new_map.info.origin.position.y ||
    !layered_costmap_->isSizeLocked()))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    RCLCPP_INFO(
      logger_,
      "StaticLayer: Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    layered_costmap_->resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x,
      new_map.info.origin.position.y,
      true);
  } else if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
    resolution_ != new_map.info.resolution ||
    origin_x_ != new_map.info.origin.position.x ||
    origin_y_ != new_map.info.origin.position.y)
  {
    // only update the size of the costmap stored locally in this layer
    RCLCPP_INFO(
      logger_,
      "StaticLayer: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    resizeMap(
      size_x, size_y, new_map.info.resolution,
      new_map.info.origin.position.x, new_map.info.origin.position.y);
  }

  unsigned int index = 0;

  // we have a new map, update full size of map
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i) {
    for (unsigned int j = 0; j < size_x; ++j) {
      unsigned char value = new_map.data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }

  m_map_frame = new_map.header.frame_id;

  m_x = m_y = 0;
  m_width = size_x_;
  m_height = size_y_;
  m_has_updated_data = true;

  current_ = true;
}

void
StaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling()) {
    nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
}

unsigned char
StaticLayer::interpretValue(unsigned char value) const
{
  return wombat_core::interpret_occupancy_to_costmap_value(
    value,
    m_trinary_costmap,
    m_unknown_cost_value,
    m_track_unknown_space,
    m_lethal_threshold);
}

void
StaticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!m_map_received) {
    m_map_received_in_update_bounds = false;
    return;
  }
  m_map_received_in_update_bounds = true;

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  // If there is a new available map, load it.
  if (m_map_buffer) {
    processMap(*m_map_buffer);
    m_map_buffer = nullptr;
  }

  if (!layered_costmap_->isRolling() ) {
    if (!(m_has_updated_data || has_extra_bounds_)) {
      return;
    }
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx = 0.0;
  double wy = 0.0;

  mapToWorld(m_x, m_y, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(m_x + m_width, m_y + m_height, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  m_has_updated_data = false;
}

void
StaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }
  if (!m_map_received_in_update_bounds) {
    static int s_count = 0;
    // throttle warning down to only 1/10 message rate
    if (++s_count == 10) {
      RCLCPP_WARN(logger_, "Can't update static costmap layer, no map received");
      s_count = 0;
    }
    return;
  }

  if (!layered_costmap_->isRolling()) {
    // if not rolling, the layered costmap (master_grid) has same coordinates as this layer
    if (!m_use_maximum) {
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    } else {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }
  } else {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx = 0;
    unsigned int my = 0;
    double wx = 0.0;
    double wy = 0.0;
    // Might even be in a different frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        m_map_frame, m_global_frame, tf2::TimePointZero,
        m_transform_tolerance);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "StaticLayer: %s", ex.what());
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into m_global_frame(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from m_global_frame to m_map_frame
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my)) {
          if (!m_use_maximum) {
            master_grid.setCost(i, j, getCost(mx, my));
          } else {
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
          }
        }
      }
    }
  }
  current_ = true;
}

}  // namespace wombat_core
