// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "wombat_srrg/srrg_messages/message_handlers/message_file_source.h"
#include "wombat_srrg/srrg_messages/messages/base_sensor_message.h"

namespace srrg2_core {

  MessageFileSource::~MessageFileSource() {
    this->close();
  }

  void MessageFileSource::open(const std::string& filename_) {
    param_filename.setValue(filename_);
    MessageFileSourceBase::open(filename_);
  }

  void MessageFileSource::open() {
    _deserializer.reset(new Deserializer);
    _deserializer->setFilePath(param_filename.value());
    _running = true;
  }

  void MessageFileSource::close() {
    MessageFileSourceBase::close();
    _deserializer.reset(0);
  }

  void MessageFileSource::reset() {
    open();
    this->_file_changed_flag = true;
    MessageFileSourceBase::reset();
  }

  // skips anything that is not a sensor message
  BaseSensorMessagePtr MessageFileSource::getMessage() {
    if (this->_file_changed_flag) {
      open(param_filename.value());
    }
    SerializablePtr o = nullptr;
    while (_running) {
      o = _deserializer->readObjectShared();
      if (!o) {
        _running = false;
        break;
      }

      BaseSensorMessagePtr msg = std::dynamic_pointer_cast<BaseSensorMessage>(o);
      if (msg) {
        return msg;
      }
    }
    return 0;
  }

} // namespace srrg2_core
