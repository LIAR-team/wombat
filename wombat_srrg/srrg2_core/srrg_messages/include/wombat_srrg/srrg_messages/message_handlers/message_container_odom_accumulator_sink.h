// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include <Eigen/Geometry>

#include "wombat_srrg/srrg_messages/message_handlers/message_sink_base.h"

namespace srrg2_core
{

  using TimestampIsometry3fMap =
    std::map<double,
             srrg2_core::Isometry3f,
             std::less<double>,
             Eigen::aligned_allocator<std::pair<double, srrg2_core::Isometry3f>>>;

  class MessageContainerOdomAccumulatorSink : public MessageSinkBase {
  public:
    virtual ~MessageContainerOdomAccumulatorSink() = default;

    inline void setContainer(TimestampIsometry3fMap* container_) {
      _container = container_;
    }
    inline void setTopic(const std::string topic_) {
      _topic = topic_;
    }

    inline const std::string& topic() const {
      return _topic;
    }
    inline TimestampIsometry3fMap* container() const {
      return _container;
    }

    bool putMessage(BaseSensorMessagePtr msg) override;

  protected:
    std::string _topic                 = "";
    TimestampIsometry3fMap* _container = nullptr;
  };

  using MessageContainerOdomAccumulatorSinkPtr =
    std::shared_ptr<MessageContainerOdomAccumulatorSink>;

} // namespace srrg2_core
