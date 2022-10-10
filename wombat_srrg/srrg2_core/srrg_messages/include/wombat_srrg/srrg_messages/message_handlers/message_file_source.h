// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_messages/message_handlers/message_file_source_base.h"
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core {

  class MessageFileSource : public MessageFileSourceBase {
  public:
    virtual void open(const std::string& filename) override;
    virtual void open() override;
    virtual void close() override;

    MessageFileSource() = default;
    virtual ~MessageFileSource();
    BaseSensorMessagePtr getMessage() override;
    void reset() override;

  protected:
    std::unique_ptr<Deserializer> _deserializer = nullptr;
  };

  using MessageFileSourcePtr = std::shared_ptr<MessageFileSource>;

} // namespace srrg2_core
