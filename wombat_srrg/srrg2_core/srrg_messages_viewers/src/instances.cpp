// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#include "instances.h"

#include "message_handlers/image_message_viewer.h"

namespace srrg2_core
{

void messages_registerTypes()
{
  BOSS_REGISTER_CLASS(ImageMessageViewer);
}

} // namespace srrg2_core
