#include "wombat_srrg/srrg_solver/solver_core/robustifier_policy.h"
#include "wombat_srrg/srrg_solver/solver_core/factor_base.h"

namespace srrg2_solver {

  RobustifierBasePtr RobustifierPolicyByType::getRobustifier(FactorBase* factor) {
    if (param_factor_class_name.value() == factor->className()) {
      return param_robustifier.value();
    }
    return 0;
  }

} // namespace srrg2_solver
