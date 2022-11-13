// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include <gtest/gtest.h>
#include <iostream>
#include <iterator>
#include "wombat_srrg/srrg_image/image.h"
#include "wombat_srrg/srrg_pcl/point_types.h"
#include "wombat_srrg/srrg_pcl/point_unprojector.h"

using namespace srrg2_core;
using namespace std;

using namespace srrg2_core;
using MatrixCloudPointNormalIntensity3f=srrg2_core::PointCloud_<
  Matrix_<PointNormalIntensity3f, Eigen::aligned_allocator<PointNormalIntensity3f> > >;
using Projector = PointUnprojectorPinhole_<PointNormalIntensity3fVectorCloud>;

TEST(CloudIntensity, CloudIntensity)
{
  ImageFloat depth;
  ImageFloat intensity;

  MatrixCloudPointNormalIntensity3f out;

  Projector projector;
  projector.computeMatrix<WithNormals>(out, depth, intensity);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
