// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
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
#pragma once

#include <mutex>
#include <string>
#include <vector>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace wombat_core
{

/**
 * @class StaticLayer
 * @brief Takes in an occupancy grid to add costs to costmap
 */
class StaticLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  /**
    * @brief Static Layer constructor
    */
  StaticLayer() = default;

  /**
    * @brief Static Layer destructor
    */
  ~StaticLayer() override = default;

  void setup(nav2_costmap_2d::LayeredCostmap * parent);

  /**
   * @brief Update the occupancy representation of the costmap from a new whole map.
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void update_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr new_map);

  /**
   * @brief Update the occupancy representation of the costmap from a map update.
   * @param update sub-map to update in the costmap.
   */
  void update_map(map_msgs::msg::OccupancyGridUpdate::ConstSharedPtr update);

  /**
   * @brief Reset this costmap
   */
  void reset() override;

  /**
   * @brief If clearing operations should be processed on this layer or not.
   * @return true if the layer can be cleared.
   */
  bool isClearable() override {return false;}

  /**
   * @brief Update the bounds of the master costmap by this layer's update dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x,
    double * min_y, double * max_x, double * max_y) override;

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_i X min map coord of the window to update
   * @param min_j Y min map coord of the window to update
   * @param max_i X max map coord of the window to update
   * @param max_j Y max map coord of the window to update
   */
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  /**
   * @brief Match the size of the master costmap
   */
  void matchSize() override;

private:
  /**
   * @brief Process a new map coming from a topic
   * @param new_map new map to process
   */
  void processMap(const nav_msgs::msg::OccupancyGrid & new_map);

  /**
   * @brief Interpret the value in the static map given on the topic to
   * convert into costs for the costmap to utilize
   * @param value occupancy grid value
   * @return costmap value
   */
  unsigned char interpretValue(unsigned char value) const;

  std::string m_global_frame;  ///< @brief The global frame for the costmap
  std::string m_map_frame;  /// @brief frame that map is located in

  bool m_has_updated_data{false};

  unsigned int m_x{0};
  unsigned int m_y{0};
  unsigned int m_width{0};
  unsigned int m_height{0};

  // Parameters
  bool m_track_unknown_space;
  bool m_use_maximum;
  unsigned char m_lethal_threshold;
  unsigned char m_unknown_cost_value;
  bool m_trinary_costmap;
  bool m_map_received{false};
  bool m_map_received_in_update_bounds{false};
  tf2::Duration m_transform_tolerance;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr m_map_buffer;
};

}  // namespace wombat_core
