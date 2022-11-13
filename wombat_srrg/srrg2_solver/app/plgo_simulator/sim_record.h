#pragma once
#include <memory>
#include <wombat_srrg/srrg_boss/serializable.h>
#include "wombat_srrg/srrg_solver/solver_core/instances.h"
#include "wombat_srrg/srrg_solver/solver_core/factor_graph.h"

namespace srrg2_solver {
  using namespace srrg2_core;

  struct SimRecord: public Serializable {
    int epoch;
    std::vector<size_t> vars;
    std::vector<size_t> factors;
    void serialize(ObjectData& odata, IdContext& context) override;
    void deserialize(ObjectData& odata, IdContext& context) override;
  };

  using SimRecordPtr  = std::shared_ptr<SimRecord>;
  using SimRecordList = std::list<SimRecordPtr>;

}
