// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "kennel/common/pose.hpp"

namespace kennel
{

bool pose_2d_equal(
  const geometry_msgs::msg::Pose & p1,
  const geometry_msgs::msg::Pose & p2)
{
  return p1.position.x == p2.position.x &&
    p1.position.y == p2.position.y &&
    tf2::getYaw(p1.orientation) == tf2::getYaw(p2.orientation);
}

}  // namespace kennel
