#pragma once

#include <list>
#include <wombat_srrg/srrg_boss/object_data.h>
#include <wombat_srrg/srrg_data_structures/abstract_map_view.h>
#include <wombat_srrg/srrg_geometry/ad.h>
#include <wombat_srrg/srrg_viewer/drawable_base.h>

namespace srrg2_solver {

  using namespace srrg2_core;

  class VariableDrawable : public VariableBase, public DrawableBase {
  
    /*! Draws itself on a potential canvas - overridden by different types of variables,
      to allow drawing without any stupid cast */
    void _drawImpl(ViewerCanvasPtr canvas_) const override {
      if (!canvas_)
        throw std::runtime_error("Variable::draw|invalid canvas");
      std::cerr << "VariableBase::draw not implemented" << std::endl;
    }

} // namespace srrg2_solver
