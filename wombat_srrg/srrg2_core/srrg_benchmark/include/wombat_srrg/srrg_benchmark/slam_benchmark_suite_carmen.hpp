// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "slam_benchmark_suite.hpp"
#include "wombat_srrg/srrg_config/configurable_manager.h"
#include <wombat_srrg/srrg_messages/message_handlers/message_file_source.h>
#include <wombat_srrg/srrg_messages/message_handlers/message_odom_subsampler_source.h>
#include <wombat_srrg/srrg_messages/message_handlers/message_sorted_source.h>
#include <wombat_srrg/srrg_messages/message_handlers/message_synchronized_source.h>
#include <wombat_srrg/srrg_messages/message_handlers/message_pack.h>
#include <wombat_srrg/srrg_messages/messages/point_cloud2_message.h>
#include <wombat_srrg/srrg_messages/messages/laser_message.h>

namespace srrg2_core {

  // 2D benchmark wrapper for: http://ais.informatik.uni-freiburg.de/slamevaluation/datasets.php
  class SLAMBenchmarkSuiteCARMEN : public SLAMBenchmarkSuite<Isometry2f> {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using EstimateType = Isometry2f;

    SLAMBenchmarkSuiteCARMEN() {
      _maximum_timestamp_delta_seconds = 3; // UAGGGHHGHG
      _messages.clear();
    }

    virtual ~SLAMBenchmarkSuiteCARMEN() {
    }

    void loadDataset(const std::string & filepath,
                     const size_t& number_of_message_packs_to_read_ = -1,
                     const size_t& number_of_message_pack_to_start_ = 0) override {
      _dataset_path = filepath_;

      // load AIS dataset from disk
      MessageFileSourcePtr source(new MessageFileSource());
      MessageSortedSourcePtr sorter(new MessageSortedSource());
      source->open(filepath_);
      if (!source->isOpen()) {
        throw std::runtime_error("SLAMBenchmarkSuiteCARMEN::loadDataset|unable to load dataset: " +
                                 filepath_);
      }
      sorter->param_source.setValue(source);
      sorter->param_time_interval.setValue(0.1);

      // set up synchronizer for AIS datasets
      MessageSynchronizedSourcePtr synchronizer(new MessageSynchronizedSource());
      synchronizer->param_source.setValue(sorter);
      synchronizer->param_topics.value().push_back("/FLASER");
      synchronizer->param_topics.value().push_back("/ODOM");
      synchronizer->param_time_interval.setValue(0.1);

      // parse all messages and gobble it up in one nasty vector (eats more RAM than chrome)
      size_t number_of_message_packs_read = 0;
      std::cerr << "SLAMBenchmarkSuiteCARMEN::loadDataset|buffering dataset: '" << filepath_ << "'"
                << std::endl;
      while (BaseSensorMessagePtr message = synchronizer->getMessage()) {
        MessagePackPtr message_pack = std::dynamic_pointer_cast<MessagePack>(message);
        if (!message_pack) {
          throw std::runtime_error("SLAMBenchmarkSuiteCARMEN::loadDataset|invalid message type: " +
                                   message->name());
        }

        // start gobbling up messages after the specified number (default 0, i.e. first message)
        if (number_of_message_packs_read >= number_of_message_pack_to_start_) {
          // overwrite message pack timestamp with lasers for ground truth alignment
          LaserMessagePtr laser_message(
            std::dynamic_pointer_cast<LaserMessage>(message_pack->messages[0]));
          assert(laser_message);
          message_pack->timestamp.setValue(laser_message->timestamp.value());
          _messages.push_back(message_pack);
        }

        // early termination
        if (_messages.size() == number_of_message_packs_to_read_) {
          break;
        }
        ++number_of_message_packs_read;
      }
      std::cerr << "SLAMBenchmarkSuiteCARMEN::loadDataset|loaded dataset: '" << filepath_
                << "' messages: " << _messages.size() << std::endl;

      // initialize message playback at first parsed messaged
      _current_message = _messages.begin();
    }

    void loadGroundTruth(const std::string & filepath,
                         const std::string & filepath_additional = "") override {
      _ground_truth_path = filepath_;

      // open ground truth file in AIS format: # timestamp1 timestamp2 x y z roll pitch yaw
      std::ifstream relations_file(filepath_, std::ios::in);
      if (!relations_file.good() || !relations_file.is_open()) {
        throw std::runtime_error(
          "SLAMBenchmarkSuiteCARMEN::loadGroundTruth|unable to read ground truth file: " +
          filepath_);
      }
      _relative_ground_truth_poses.clear();

      // read file by tokens - we retrieve relative transforms between a pair of poses (not
      // sorted)
      double timestamp1, timestamp2, x, y, z, roll, pitch, yaw;
      while (relations_file >> timestamp1 >> timestamp2 >> x >> y >> z >> roll >> pitch >> yaw) {
        // this dataset is 2D!
        assert(z == 0);
        assert(roll == 0);
        assert(pitch == 0);

        // add measurement as relative estimate
        Isometry2f relative_estimate(Isometry2f::Identity());
        relative_estimate.translation() = Vector2f(x, y);
        relative_estimate.rotate(yaw);
        _relative_ground_truth_poses.push_back(
          RelativeEstimateStamped(relative_estimate, timestamp1, timestamp2));
      }
      std::cerr << "SLAMBenchmarkSuiteCARMEN::loadGroundTruthFromTXT|relative poses: "
                << _relative_ground_truth_poses.size() << std::endl;
      relations_file.close();
    }

    BaseSensorMessagePtr getMessage() override {
      // check if dataset has been completed
      if (_current_message == _messages.end()) {
        return nullptr;
      }

      // retrieve the current message and move the iterator for the next call
      BaseSensorMessagePtr message =
        std::dynamic_pointer_cast<BaseSensorMessage>(*_current_message);
      ++_current_message;
      return message;
    }

    void reset() override {
      _current_message = _messages.begin();
    }

    void writeTrajectoryToFile(const std::string& filename_ = "trajectory.txt") const override {
      // TODO implement me
    }

  protected:
    // parsed messages (in packs)
    std::vector<MessagePackPtr> _messages;
    std::vector<MessagePackPtr>::iterator _current_message;
  };

} // namespace srrg2_core
