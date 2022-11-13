#include "wombat_srrg/srrg_solver/utils/factor_graph_initializer_rule.h"
#include "wombat_srrg/srrg_solver/utils/factor_graph_initializer.h"

namespace srrg2_solver {
  FactorGraphInitializerRuleBase::FactorGraphInitializerRuleBase(FactorGraphInitializer* initializer_) {
    _initializer=initializer_;
  }

}
