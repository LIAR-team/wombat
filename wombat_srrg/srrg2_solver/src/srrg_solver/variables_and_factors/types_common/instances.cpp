//! include all the types you declared here
#include "instances.h"
#include "all_types.h"
#include "srrg_solver/solver_core/variable_impl.cpp"
#include "variable_vector_impl.cpp"
#include "variable_vector_ad_impl.cpp"
#include "variable_matrix_impl.cpp"
#include "variable_matrix_ad_impl.cpp"

#include <srrg_solver/solver_core/instance_macros.h>
//! we try to instantiate all solvers
namespace srrg2_solver {
  using namespace srrg2_core;

  void VariableVector3::_drawImpl(ViewerCanvasPtr canvas_) const {
    if (!canvas_)
      throw std::runtime_error("VariablePoint3::draw|invalid canvas");
    canvas_->pushColor();
    canvas_->setColor(srrg2_core::ColorPalette::color3fGreen());
    canvas_->putPoints(1, &_estimate);
    canvas_->popAttribute();
  }
  
  INSTANTIATE(VariableVector2);
  INSTANTIATE(VariableVector3);
  INSTANTIATE(VariableVector4);
  INSTANTIATE(VariableVector5);
  INSTANTIATE(VariableVector6);
  INSTANTIATE(VariableVector7);
  INSTANTIATE(VariableVector8);

  INSTANTIATE(VariableVector2AD);
  INSTANTIATE(VariableVector3AD);
  INSTANTIATE(VariableVector4AD);
  INSTANTIATE(VariableVector5AD);
  INSTANTIATE(VariableVector6AD);
  INSTANTIATE(VariableVector7AD);
  INSTANTIATE(VariableVector8AD);
  INSTANTIATE(VariableMatrix3_4);
  INSTANTIATE(VariableMatrix3_4AD);
  
  // this is the function you have to call to initialize
  // the serialization subsystem
  void variables_and_factors_common_registerTypes() {
    BOSS_REGISTER_CLASS(VariablePoint2);
    BOSS_REGISTER_CLASS(VariablePoint3);
    BOSS_REGISTER_CLASS(VariableVector4);
    BOSS_REGISTER_CLASS(VariableVector5);
    BOSS_REGISTER_CLASS(VariableVector6);
    BOSS_REGISTER_CLASS(VariableVector7);
    BOSS_REGISTER_CLASS(VariableVector8);

    BOSS_REGISTER_CLASS(VariablePoint2AD);
    BOSS_REGISTER_CLASS(VariablePoint3AD);
    BOSS_REGISTER_CLASS(VariableVector4AD);
    BOSS_REGISTER_CLASS(VariableVector5AD);
    BOSS_REGISTER_CLASS(VariableVector6AD);
    BOSS_REGISTER_CLASS(VariableVector7AD);
    BOSS_REGISTER_CLASS(VariableVector8AD);
    BOSS_REGISTER_CLASS(VariableMatrix3_4);
    BOSS_REGISTER_CLASS(VariableMatrix3_4AD);
  }
} // namespace srrg2_solver
