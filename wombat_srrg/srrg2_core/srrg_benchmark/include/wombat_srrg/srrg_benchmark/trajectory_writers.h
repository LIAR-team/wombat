// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include <map>
#include <string>

#include "wombat_srrg/srrg_geometry/geometry_defs.h"

using TimestampIsometry3fMap =
  std::map<double,
           srrg2_core::Isometry3f,
           std::less<double>,
           Eigen::aligned_allocator<std::pair<double, srrg2_core::Isometry3f>>>;

void writeTrajectoryToFileTUM(
  const TimestampIsometry3fMap & conatiner,
  const std::string & filename);

void writeTrajectoryToFileKITTI(
  const TimestampIsometry3fMap & conatiner,
  const std::string & filename);
