// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include <gtest/gtest.h>

#include "wombat_srrg/srrg_path_matrix/path_matrix_distance_search.h"
#include "wombat_srrg/srrg_image/image.h"
#include "wombat_srrg/srrg_pcl/point_types.h"
#include "wombat_srrg/srrg_system_utils/system_utils.h"

using namespace srrg2_core;
using namespace std;

// #DEFINE VISUALIZATION

TEST(DistanceMap, DistanceMap)
{
  int rows       = 480;
  int cols       = 640;
  int num_points = 100;

  // create a container with the random points;
  Point2fVectorCloud points;
  points.resize(num_points);
  for (size_t i = 0; i < points.size(); ++i) {
    Point2f p;
    p.coordinates() = Vector2f(rows * drand48(), cols * drand48());
    points[i]       = p;
  }

  PathMatrix distance_map(rows, cols);
  ImageUInt8 path_map_image;
  ImageInt parent_map_image;
  cv::Mat m, p;
  const int d_max = 100;
#ifdef VISUALIZATION
  cv::namedWindow("distance map");
  cv::namedWindow("parent map");
#endif

  int d_curr = 0;

  PathMatrixDistanceSearch dmap_calculator;

  dmap_calculator.setPathMatrix(&distance_map);
  // loop over several max_distance_squared_pxl to let you see that the distance map is growing
  // for visualization purpose
  while (d_curr < d_max) {
    dmap_calculator.param_max_distance_squared_pxl.setValue(d_curr * d_curr);
    double t_start = getTime();
    dmap_calculator.setGoals(points);
    dmap_calculator.compute();
    double t_end = getTime();
    distance_map.toImage(path_map_image, PathMatrix::Distance);
    distance_map.toImage(parent_map_image, PathMatrix::Parent);
    path_map_image.toCv(m);
    parent_map_image.toCv(p);

#ifdef VISUALIZATION
    cv::imshow("distance map", m);
    cv::imshow("parent map", p);
    cv::waitKey();
#endif

    std::cout << "dist: " << d_curr << " time: " << t_end - t_start << std::endl;
    d_curr++;
  }
  cout << "terminating" << endl;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
