// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/message_handlers/message_file_source_base.h"
#include "wombat_srrg/srrg_config/configurable.h"
#include "wombat_srrg/srrg_system_utils/system_utils.h"

namespace srrg2_core {

  void MessageFileSourceBase::open(const std::string& filename_) {
    if (_is_open) {
      this->close();
    }
    if (!isAccessible(filename_)) {
      std::string msg = filename_ + " not found";
      throw std::runtime_error(msg.c_str());
    }
    param_filename.setValue(filename_);
    open();
    _is_open           = true;
    _file_changed_flag = false;
  }

  MessageSourceBase* MessageFileSourceBase::getRootSource() {
    return this;
  }

  void MessageFileSourceBase::close() {
    if (!_is_open) {
      return;
    }
    _is_open = false;
  }

} // namespace srrg2_core
