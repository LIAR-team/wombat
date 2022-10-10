#include "constraint_factor.h"
#include "factor_impl.cpp"
#include "factor_updaters.h"
#include <Eigen/Dense>
namespace srrg2_solver {
  using namespace srrg2_core;

  template <int ConstraintDim_, typename... VariableTypes_>
  bool ConstraintFactor_<ConstraintDim_, VariableTypes_...>::isValid() const {
    return _is_valid;
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::compute(bool chi_only, bool force) {
    this->_stats.status               = FactorStats::Status::Suppressed;
    this->_stats.chi                  = 0;
    this->_stats.constraint_violation = 0;

    if (!this->isActive() && !force) {
      return;
    }
    constraintAndJacobian(chi_only);
    if (!isValid()) {
      return;
    }
    float constraint_violation = 0;
    for (int i = 0; i < ConstraintDim_; ++i) {
      switch (this->_constraint_type) {
        case Equality:
          // bb whatever value different from zero is a violation
          constraint_violation = std::max(fabs(_constraint(i)), constraint_violation);
          break;
        case Inequality:
          // bb only positive values of the contraint are violations
          constraint_violation = std::max(_constraint(i), constraint_violation);
          break;
        default:
          throw std::runtime_error("constrant_factor_impl: please set the FactorConstraintType");
      }
    }
    this->_stats.constraint_violation = constraint_violation;
    ConstraintVectorType m =
      _multiplier + _rho * (/*_J * this->variables().lastPerturbation()+*/ _constraint);
    switch (this->_constraint_type) {
      case Equality:
        _multiplier = m;
        break;
      case Inequality:
        for (int i = 0; i < ConstraintDim; ++i) {
          _multiplier[i] = std::max(0.f, m[i]);
        }
        break;
      default:
        throw std::runtime_error("constrant_factor_impl: please set the FactorConstraintType");
    }
    if (chi_only) {
      return;
    }
    // std::cerr << _J << std::endl;
    updateH();
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  template <int r, int c>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::_updateHBlock() {
    const int BlockRows = ThisType::template perturbationDim<r>();
    const int BlockCols = ThisType::template perturbationDim<c>();
    int idx             = BaseType::template blockOffset<r, c>();
    // a block is null when a variable is fixed
    if (!this->_H_blocks[idx]) {
      return;
    }
    if (!this->_H_transpose[idx]) {
      assert(this->_H_blocks[idx]->rows() == BlockRows &&
             this->_H_blocks[idx]->cols() == BlockCols && "dimension mismatch");
      Eigen::Map<Eigen::Matrix<float, BlockRows, BlockCols>> target_H(
        this->_H_blocks[idx]->storage());
      target_H.noalias() += this->_rho * ThisType::template jacobian<r>().transpose() *
                            _weighting_matrix * ThisType::template jacobian<c>();

    } else {
      assert(this->_H_blocks[idx]->rows() == BlockCols &&
             this->_H_blocks[idx]->cols() == BlockRows && "dimension mismatch");

      Eigen::Map<Eigen::Matrix<float, BlockCols, BlockRows>> target_H(
        this->_H_blocks[idx]->storage());
      target_H.noalias() += this->_rho * ThisType::template jacobian<c>().transpose() *
                            _weighting_matrix * ThisType::template jacobian<r>();
    }
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  template <int r>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::_updateBBlock() {
    const int BlockRows = ThisType::template perturbationDim<r>();
    using namespace std;
    if (!this->_b_blocks[r]) {
      // a block is null if the variable is fixed
      return;
    }
    Eigen::Map<Eigen::Matrix<float, BlockRows, 1>> target_b(this->_b_blocks[r]->storage());
    assert(this->_b_blocks[r]->rows() == BlockRows && "dimension mismatch");
    target_b.noalias() -= ThisType::template jacobian<r>().transpose() *
                          (this->_multiplier + this->_rho * this->_constraint);
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::updateH() {
    RowUpdater<ThisType, NumVariables - 1, NumVariables - 1>::update(*this);
  }

  // -------------------------------------------------------------------------------------------
  // //
  // -------------------------------------------------------------------------------------------
  // //
  // -------------------------------------------------------------------------------------------
  // //
  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::serialize(ObjectData& odata,
                                                                       IdContext& context) {
    Identifiable::serialize(odata, context);
    odata.setInt("graph_id", BaseType::graphId());
    odata.setBool("enabled", BaseType::enabled());
    odata.setInt("level", this->level());

    ArrayData* adata = new ArrayData;
    for (int pos = 0; pos < NumVariables; ++pos) {
      adata->add((int) BaseType::variableId(pos));
    }
    odata.setField("variables", adata);
    odata.setFloat("rho", _rho);
    odata.setFloat("multiplier_initialization", _multiplier_initialization);
    odata.setInt("constraint_type", (int) _constraint_type);

    int w_rows       = _weighting_matrix.rows();
    int w_cols       = _weighting_matrix.cols();
    ArrayData* wdata = new ArrayData;
    for (int r = 0; r < w_rows; ++r) {
      for (int c = r; c < w_cols; ++c) {
        wdata->add(_weighting_matrix(r, c));
      }
    }
    odata.setField("weights", wdata);
  }

  template <int ConstraintDim_, typename... VariableTypes_>
  void ConstraintFactor_<ConstraintDim_, VariableTypes_...>::deserialize(ObjectData& odata,
                                                                         IdContext& context) {
    Identifiable::deserialize(odata, context);
    FactorBase::_graph_id = odata.getInt("graph_id");
    if (odata.getField("enabled")) {
      FactorBase::_enabled = odata.getBool("enabled");
    }
    if (odata.getField("level")) {
      FactorBase::setLevel(odata.getInt("level"));
    }
    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("variables"));
    int pos          = 0;
    for (auto it = adata->begin(); it != adata->end(); ++it) {
      ThisType::_variables.setGraphId(pos, (*it)->getInt());
      ++pos;
    }

    _rho                       = odata.getFloat("rho");
    _multiplier_initialization = odata.getFloat("multiplier_initialization");
    _constraint_type           = (FactorConstraintType) odata.getInt("constraint_type");

    int w_rows       = _weighting_matrix.rows();
    int w_cols       = _weighting_matrix.cols();
    ArrayData* wdata = dynamic_cast<ArrayData*>(odata.getField("weights"));
    int k            = 0;
    for (int r = 0; r < w_rows; ++r) {
      for (int c = r; c < w_cols; ++c, ++k) {
        _weighting_matrix(r, c) = (*wdata)[k].getFloat();
        _weighting_matrix(c, r) = _weighting_matrix(r, c);
      }
    }
  }

} // namespace srrg2_solver
