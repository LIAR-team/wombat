// Copyright 2024 Soragna Alberto.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"

#include "kennel/sim_time_manager.hpp"
#include "wombat_core/math/statistics.hpp"

using namespace std::chrono_literals;

class TestSimTimeManager : public testing::Test
{
public:
  void SetUp() override
  {
    rclcpp::init(0, nullptr);
    rclcpp::NodeOptions node_options;
    node_options.use_intra_process_comms(true);
    node = std::make_shared<rclcpp::Node>("test_node", node_options);
  }

  void TearDown() override
  {
    rclcpp::shutdown();
  }

  std::shared_ptr<rclcpp::Node> node;
};

struct test_data_t
{
  double rtf;
  std::chrono::milliseconds sim_time_update_period;
  double measured_rtf_tolerance;
  size_t num_messages {100};
  std::chrono::seconds max_duration {10s};
};

class TestSimTimeManagerRTFWithParams
  : public TestSimTimeManager, public ::testing::WithParamInterface<test_data_t>
{};

INSTANTIATE_TEST_SUITE_P(
  TestMeasuredRTF,
  TestSimTimeManagerRTFWithParams,
  ::testing::Values(
    // RTF 1
    test_data_t {
  1.0,  // rtf
  1ms,  // sim_time_update_period
  0.15,  // rtf tolerance
  500,  // num samples
},
    // RTF > 1
    test_data_t {
  2.0,  // rtf
  1ms,  // sim_time_update_period
  0.25,  // rtf tolerance
  500,  // num samples
},
    // RTF >> 1
    test_data_t {
  15.0,  // rtf
  1ms,  // sim_time_update_period
  2.5,  // rtf tolerance
  500,  // num samples
},
    // RTF < 1
    test_data_t {
  0.5,  // rtf
  1ms,  // sim_time_update_period
  0.15,  // rtf tolerance
  500,  // num samples
}
  ),
  [](const ::testing::TestParamInfo<TestSimTimeManagerRTFWithParams::ParamType> & my_info)
  {
    // If the rtf is 3.456 this will return "rtf3o45"
    double whole = 0.0;
    double fractional = std::modf(my_info.param.rtf, &whole);
    return "rtf" + std::to_string(static_cast<int>(whole)) + "o" + std::to_string(static_cast<int>(fractional * 100));
  });

TEST_P(TestSimTimeManagerRTFWithParams, MeasuredRTF)
{
  auto test_params = GetParam();

  rosgraph_msgs::msg::Clock::ConstSharedPtr last_msg;
  auto last_msg_steady_time = std::chrono::steady_clock::now();
  std::vector<double> collected_rtf;
  collected_rtf.reserve(test_params.num_messages);

  auto clock_subscription = node->create_subscription<rosgraph_msgs::msg::Clock>(
    "/clock",
    rclcpp::SensorDataQoS().keep_last(1),
    [&](rosgraph_msgs::msg::Clock::ConstSharedPtr msg)
    {
      auto now = std::chrono::steady_clock::now();
      if (last_msg) {
        const rclcpp::Duration sim_time_delta = rclcpp::Time(msg->clock) - rclcpp::Time(last_msg->clock);
        const auto steady_time_delta = now - last_msg_steady_time;
        const int64_t steady_time_delta_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(steady_time_delta).count();
        const double measured_rtf =
        static_cast<double>(sim_time_delta.nanoseconds()) / static_cast<double>(steady_time_delta_ns);
        collected_rtf.push_back(measured_rtf);
      }
      last_msg = msg;
      last_msg_steady_time = now;
    }
  );

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  bool spin_exited = false;
  std::thread executor_thread(
    [&spin_exited, &executor]()
    {
      executor.spin();
      spin_exited = true;
    });

  auto sim_time_manager = std::make_shared<kennel::SimTimeManager>(
    node.get(),
    test_params.rtf,
    test_params.sim_time_update_period);

  std::thread sim_time_thread(
    [&sim_time_manager]()
    {
      sim_time_manager->run();
    });

  // Wait some time for the subscription to receive the messages
  auto start = std::chrono::high_resolution_clock::now();
  while (
    collected_rtf.size() < test_params.num_messages &&
    !spin_exited &&
    (std::chrono::high_resolution_clock::now() - start < test_params.max_duration))
  {
    std::this_thread::sleep_for(25ms);
  }

  sim_time_manager->stop();
  executor.cancel();

  executor_thread.join();
  sim_time_thread.join();

  EXPECT_GE(collected_rtf.size(), test_params.num_messages);

  const size_t previous_size = collected_rtf.size();
  wombat_core::remove_outliers(collected_rtf);

  ASSERT_GT(collected_rtf.size(), previous_size / 2);
  EXPECT_NEAR(test_params.rtf, wombat_core::compute_mean(collected_rtf), test_params.measured_rtf_tolerance);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
