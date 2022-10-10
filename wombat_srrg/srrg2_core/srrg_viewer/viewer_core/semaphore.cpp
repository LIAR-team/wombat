// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "semaphore.h"

namespace srrg2_core {

  Semaphore::Semaphore(const uint64_t& value_) {
    _count = value_;
  }

  Semaphore::~Semaphore() {}

} //ia end namespace
