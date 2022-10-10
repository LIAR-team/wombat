// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include "object_writer.h"

namespace srrg2_core {

class JSONObjectWriter: public ObjectWriter {
public:
  virtual void writeObject(std::ostream& os,
                           const std::string& type,
                           ObjectData& object,
                           bool comments_enabled=false);
protected:
  int indentation_level=0;
};

}
