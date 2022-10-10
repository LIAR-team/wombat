// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "wombat_srrg/srrg_messages/message_handlers/message_source_base.h"
#include "wombat_srrg/srrg_config/configurable.h"
#include "wombat_srrg/srrg_property/property.h"
#include <thread>

namespace srrg2_core {

  class MessageFileSourceBase : public MessageSourceBase {
    // configuration
  public:
    PARAM(PropertyString, filename, "file to read", "", &_file_changed_flag);

  public:
    MessageFileSourceBase() : MessageSourceBase(){};
    virtual ~MessageFileSourceBase() = default;

    MessageSourceBase* getRootSource() override;

    virtual void open(const std::string& filename);
    virtual void open() override = 0;
    virtual void close();

  protected:
    bool _file_changed_flag = true;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  using MessageFileSourceBasePtr = std::shared_ptr<MessageFileSourceBase>;

} // namespace srrg2_core
