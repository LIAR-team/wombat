#pragma once
#include "factor.h"

namespace srrg2_solver {

  enum FactorConstraintType { Equality = 0x0, Inequality = 0x1 };

  template <int ConstraintDim_, typename... VariableTypes_>
  class ConstraintFactor_ : public Factor_<VariablePtrTuple_<VariableTypes_...>> {
  public:
    using ThisType = ConstraintFactor_<ConstraintDim_, VariableTypes_...>;

    template <typename T_>
    friend class StarFactorCreator_;

    using VariableTupleType = VariablePtrTuple_<VariableTypes_...>;
    using BaseType          = Factor_<VariableTupleType>;

    static constexpr int NumVariables         = BaseType::NumVariables;
    static constexpr int ConstraintDim        = ConstraintDim_;
    using ConstraintVectorType                = Eigen::Matrix<float, ConstraintDim, 1>;
    static constexpr int TotalPerturbationDim = BaseType::TotalPerturbationDim;
    using TotalPerturbationVectorType         = typename BaseType::TotalPerturbationVectorType;

    using TotalJacobianMatrixType =
      typename Eigen::Matrix<float, ConstraintDim, TotalPerturbationDim>;

    using WeightingMatrixType = Eigen::Matrix<float, ConstraintDim, ConstraintDim>; /*!< Type of the
                                                                                 weighting matrix */

    template <typename ThisType_, int r, int c>
    friend struct ColUpdater;

    template <typename ThisType_, int r, int c>
    friend struct RowUpdater;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    /*! @return Dimension of the i-th variable*/
    template <int i>
    static constexpr int perturbationDim() {
      return VariableTupleType::template perturbationDim<i>();
    }

    /*! @return Offset of the i-th variable perturbation in the jacobian */
    template <int i>
    static constexpr int perturbationOffset() {
      return VariableTupleType::template perturbationOffset<i>();
    }

    /*! @return Type of the i-th jacobian */
    template <int i>
    using JacobianMatrixType = typename Eigen::Map<
      const Eigen::Matrix<float, ConstraintDim, ThisType::template perturbationDim<i>()>>;

    /*! @return The i-th jacobian (read only) */
    template <int i>
    inline const JacobianMatrixType<i> jacobian() const {
      return JacobianMatrixType<i>(_J.data() +
                                   ThisType::template perturbationOffset<i>() * ConstraintDim);
    }
    /*! Type of the i-th jacobian */
    template <int i>
    using MutableJacobianMatrixType = typename Eigen::Map<
      Eigen::Matrix<float, ConstraintDim, ThisType::template perturbationDim<i>()>>;

    /*! @return The i-th jacobian */
    template <int i>
    inline MutableJacobianMatrixType<i> jacobian() {
      return MutableJacobianMatrixType<i>(_J.data() + ThisType::template perturbationOffset<i>() *
                                                        ConstraintDim);
    }

    /*! @return measurement dim */
    int measurementDim() const final {
      return ConstraintDim;
    }

    /*! @return The error vector */
    inline const ConstraintVectorType& constraint() const {
      return _constraint;
    }

    /*! @return The total jacobian */
    inline const TotalJacobianMatrixType& totalJacobian() const {
      return _J;
    }

    /*! @return Weighting matrix (read only)*/
    inline const WeightingMatrixType& weightingMatrix() const {
      return _weighting_matrix;
    }

    /*! Setter for the information matrix
     @param[in] information_matrix inverse covariance matrix to be assigned to the factor
     */
    inline void setWeightingMatrix(const WeightingMatrixType& weighting_matrix) {
      _weighting_matrix = weighting_matrix;
    }

    /*! Checks if the computation is good for this factor */
    bool isValid() const override;

    virtual void constraintAndJacobian(bool error_only) = 0;

    void compute(bool chi_only = false, bool force = false) override;

    /*! Serialize the measurement contained in the factor */
    void serialize(ObjectData& odata, IdContext& context) override;
    /*! Deserialize the measurement contained in the factor */
    void deserialize(ObjectData& odata, IdContext& context) override;

    /*! Auxiliary function to update the approximate hessian blocks that corresponds
      to the variables involved in the factor */
    template <int r, int c>
    inline void _updateHBlock();

    /*! Auxiliary function to update the gradient vector blocks that corresponds
      to the variables involved in the factor */
    template <int r>
    inline void _updateBBlock();

    /*! Auxiliary function to update the approximate hessian that corresponds
      to the variables involved in the factor */
    inline void updateH();

    enum FactorConstraintType _constraint_type = Equality;

    float rho() const {
      return _rho;
    }

    void setRho(const float& rho_) {
      _rho = rho_;
    }

    float multiplierInitalization() const {
      return _multiplier_initialization;
    }

    void setMultiplierInitalization(const float& multiplier_initalization_) {
      _multiplier_initialization = multiplier_initalization_;
    }

  protected:
    float _rho                       = 0.1; // gain for updating the multiplier
    float _multiplier_initialization = 20;
    ConstraintVectorType _constraint =
      ConstraintVectorType::Zero(); // residual of the constraint evaluation 0 or <=0
    ConstraintVectorType _multiplier =
      _multiplier_initialization * ConstraintVectorType::Ones(); // scale, updated before
                                                                 // compute
    TotalJacobianMatrixType _J            = TotalJacobianMatrixType::Zero(); // d_constraint/d_x
    WeightingMatrixType _weighting_matrix = WeightingMatrixType::Identity();
    bool _is_valid                        = true;
  };

} // namespace srrg2_solver

#include "constraint_factor_impl.hpp"
