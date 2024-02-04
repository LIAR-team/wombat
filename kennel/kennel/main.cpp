// Copyright 2024 Soragna Alberto.
// All Rights Reserved.
// Unauthorized copying via any medium is strictly prohibited.
// Proprietary and confidential.

#include <condition_variable>
#include <mutex>

#include "rclcpp/rclcpp.hpp"

#include "kennel/kennel.hpp"

// TODO: implement a clean shutdown
static void wait_until_shutdown()
{
  std::mutex ros_shutdown_mutex;
  std::condition_variable ros_shutdown_cv;
  rclcpp::on_shutdown(
    [&]() {
      std::cout << "Shutdown!" << std::endl;
      ros_shutdown_cv.notify_one();
    });

  std::unique_lock lock(ros_shutdown_mutex);
  ros_shutdown_cv.wait(
    lock,
    []() {
      std::cout << "Shutdown wait done" << std::endl;
      return !rclcpp::ok();
    });

  rclcpp::shutdown();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto kennel = std::make_shared<kennel::Kennel>();
  kennel->start();

  wait_until_shutdown();

  return 0;
}
