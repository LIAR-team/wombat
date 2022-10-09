#include "instances.h"

#include "message_handlers/image_message_viewer.h"


namespace srrg2_core
{
  void messages_registerTypes()
  {
    BOSS_REGISTER_CLASS(ImageMessageViewer);
  }
} // namespace srrg2_core
