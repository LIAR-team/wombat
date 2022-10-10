// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once
#include "message_sink_base.h"
#include "wombat_srrg/srrg_viewer/active_drawable.h"

namespace srrg2_core {
  class ImageMessageViewer : public MessageSinkBase, public ActiveDrawable  {
  public:
    PARAM_VECTOR(PropertyVector_<std::string>,
                 image_topics,
                 "image topics to show",
                 nullptr);

    ImageMessageViewer()          = default;

    virtual ~ImageMessageViewer() = default;

    bool putMessage(BaseSensorMessagePtr msg_) override;

    void reset() override;

  protected:
    void _drawImpl(ViewerCanvasPtr canvas) const override;
    std::vector<cv::Mat> _images;
  };

  using MessageSinkBasePtr = std::shared_ptr<MessageSinkBase>;
} // namespace srrg2_core
