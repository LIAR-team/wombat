#pragma once

#include <list>
#include <set>

#include <wombat_srrg/srrg_boss/object_data.h>
#include <wombat_srrg/srrg_data_structures/abstract_map_view.h>
#include <wombat_srrg/srrg_geometry/ad.h>

namespace srrg2_solver {

using namespace srrg2_core;

/*! @brief Base class of a variable.
    A variable is characterized by:
    - a unique graph id (-1 when invalid)
    - a status (active when partecipate to the optimization, fixed otherwise)
    - an hessian index which correspond to the block of the approximate hessian that correspond to
  this variable
    - an updated status  that tells if the variable has been recently changed

    A variable implements
    - a perturbationDim() method, that returns the dimension of the
      perturbation
    - a method to apply the perturbation from an array of float
      (having at minimum dimension perturbationDim)
    - a setZero() method that bring the variable to the origin of its domain
    - push() and pop() method which manage a stack of estimate values of the variable at different
  steps
*/
class VariableBase : public Identifiable
{
public:
  enum Status { Active = 0, NonActive = 1, Fixed = 2 };
  using Id = int64_t;

protected:
  friend class Solver;
  template <typename V, typename... VRest>
  friend struct VariablePtrTuple_;

public:
  /*! Set the value of the variable to the origin of its domain */
  virtual void setZero() = 0;

  /*! Returns the dimension of the perturbation */
  virtual int perturbationDim() const = 0;

  /*! Applies the perturbation to the variable
    @param[in] pert_data perturbation vector represented as a C-style array, the dimension must be
    at least perturbationDim
    */
  virtual void applyPerturbationRaw(const float* pert_data) = 0;

  /*! Returns the graph id */
  inline Id graphId() const
  {
    return _graph_id;
  }

  /*! Sets the id */
  inline void setGraphId(Id graph_id_)
  {
    _graph_id = graph_id_;
  }

  /*! Queries the status */
  inline Status status() const
  {
    return _status;
  }

  /*! Sets the status */
  inline void setStatus(Status status_)
  {
    _status = status_;
  }

  /*! Pushes the current estimate to the stack */
  virtual void push() = 0;

  /*! Pops the current estimate from the stack */
  virtual void pop() = 0;

  /*! Keeps the current estimate, but discards the top of the stack
    throws away the most recently saved value*/
  virtual void discardTop() = 0;

  /*! Get tainted status */
  inline bool updated() const
  {
    return _updated;
  }
  /*! Untaint variable */
  inline void clearUpdated()
  {
    _updated = false;
  }

  inline int hessianIndex() const
  {
    return _hessian_index;
  }

  virtual void copyEstimate(const VariableBase& src) = 0;

protected:
  /*! Serialization id getter (not related to graph id) */
  inline int getId()
  {
    return Identifiable::getId();
  }
  /*! Set serialization id */
  inline void setId(int id_)
  {
    Identifiable::setId(id_);
  }
  virtual void normalize()
  {}

  Id _graph_id       = -1;
  int _hessian_index = -1;
  Status _status     = Active;
  bool _updated      = true;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using VariableBasePtr = std::shared_ptr<VariableBase>;

using IdSet = std::set<VariableBase::Id>;

/*! @brief Intermediate variable class used for generic
    non-Eigen estimate types. Serialization and Deserialization must be specified by the user.
    A VariableGeneric_ is defined using two templates :
    - PerturbationDim_ specify the dimension of the Euclidian perturbation
    - EstimateType_ is the internal type of the variable, this must be an partially instatiated
    template on the scalar type  - e.g. EstimateType_<T> - which is then automatically selected as
    float DualValuef (used for Autodiff).
    - The estimate type MUST define a static method called Identity() to inizialize its value

    To define a variable starting from VariableGeneric_ one must override :
    - setZero()
    - applyPerturbation(), which takes as argument an Eigen vector, not a C-style array
    - [optional] serialize() and deserialize() methods
*/
template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
class VariableGeneric_ : public VariableBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using BaseVariableType               = VariableGeneric_<PerturbationDim_, EstimateType_>;
  using EstimateType                   = EstimateType_<float>;      // this sucks
  using ADEstimateType                 = EstimateType_<DualValuef>; // this sucks
  static constexpr int PerturbationDim = PerturbationDim_;
  using PerturbationVectorType =
    typename Eigen::Matrix<float, PerturbationDim_, 1>; /*!< Euclidian perturbation of the
                                                            variable */
  PerturbationVectorType last_perturbation;
  /*! Set the variable to the origin of the domain on construction */
  VariableGeneric_()
  {
    last_perturbation.setZero();
  }
  /*! Virtual destructor */
  virtual ~VariableGeneric_(){};

  /*! Get perturbation dimension */
  int perturbationDim() const override
  {
    return PerturbationDim;
  }

  /*! Method of the base class, that calls the typed
      applyPerturbation of VariableGeneric_ */
  void applyPerturbationRaw(const float* pert_data_) override
  {
    Eigen::Map<const PerturbationVectorType> pert(pert_data_);
    last_perturbation = pert;
    applyPerturbation(pert);
  }

  virtual void setZero() = 0;

  /*! Set the estimate value for the variable*/
  virtual void setEstimate(const EstimateType& est_)
  {
    this->_estimate    = est_;
    this->_updated     = true;
    //this->_need_redraw = true;
  }

  /*! Accessor for the estimate value */
  inline const EstimateType& estimate() const
  {
    return _estimate;
  }

  void copyEstimate(const VariableBase& src) override
  {
    const BaseVariableType& src_casted = dynamic_cast<const BaseVariableType&>(src);
    setEstimate(src_casted.estimate());
    setStatus(src_casted.status());
  }

  /*! To be defined in the derived classes
    @param[in] perturbation_ perturbation vector to be applied on the current estimate (Eigen
    typed)
    */
  virtual void applyPerturbation(const PerturbationVectorType& perturbation_) = 0;

  /*! Pushes the variable on the variable stack
          (like openGL does with attributes and matrices)
  */
  void push() override;

  /*! pops the variable from the variable stack
  (like openGL does with attributes and matrices) */
  void pop() override;

  /*! Keeps the current estimate, but discards the top of the stack
          (throws away the most recently saved value)
  */
  void discardTop() override;

  /*! Serialization of the variable through BOSS.
      This must be specialized in the derived class
  */
  void serialize(ObjectData & odata, IdContext & context) override
  {
    (void)odata;
    (void)context;
    throw std::runtime_error("VariableGeneric_::serialize|you must implement this function");
  }

  /*! Deserialization of the variable through BOSS.
      This must be specialized in the derived class
  */
  void deserialize(ObjectData& odata, IdContext& context) override
  {
    (void)odata;
    (void)context;
    throw std::runtime_error("VariableGeneric_::deserialize|you must implement this function");
  }

protected:
  EstimateType _estimate =
    EstimateType::Identity(); /*!< Estimate value of the variable, initialized as identity*/
  std::list<EstimateType, Eigen::aligned_allocator<EstimateType>>
    _stack; /*!< Stack of variable values,
              push() and pop() interact with it */
};

/*! @brief Variable with Eigen-type estimate.
    Serialization and Deserialization comes for free.
    A Variable_ is defined using two templates :
    - PerturbationDim_ specify the dimension of the Euclidian perturbation
    - EstimateType_ is the internal type of the variable, this must be an partially instatiated
    template on the scalar type  - e.g. EstimateType_<T> - which is then automatically selected as
    float and DualValuef (used for Autodiff).

    To define a variable starting from Variable_ one must override :
    - setZero()
    - applyPerturbation(), which takes as argument an Eigen vector, not a C-style array
*/
template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
class Variable_ : public VariableGeneric_<PerturbationDim_, EstimateType_> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BaseVariableType = Variable_<PerturbationDim_, EstimateType_>;
  using EstimateType     = EstimateType_<float>;

  using ADEstimateType = EstimateType_<DualValuef>;

  static const int PerturbationDim = PerturbationDim_;
  using PerturbationVectorType =
    typename Eigen::Matrix<float, PerturbationDim_, 1>; /*!< Eigen perturbation vector */

  //! @brief object life, do nothing
  Variable_() = default;

  virtual ~Variable_() = default;

  /*! Must be overrided in the derived class */
  void setZero() override
  {}

  /*! Must be overrided in the derived class */
  void applyPerturbation(const PerturbationVectorType & perturbation) override
  {
    (void)perturbation;
  }

  /*!Serialization assuming eigen estimate - implemented in the _impl.cpp
    this CANNOT be overridden to avoid strange behaviours */
  void serialize(ObjectData& odata, IdContext& context);

  /*! Deserialization assuming eigen estimate - implemented in the _impl.cpp
    this CANNOT be overridden to avoid strange behaviours */
  void deserialize(ObjectData& odata, IdContext& context);
};

template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
void VariableGeneric_<PerturbationDim_, EstimateType_>::push()
{
  _updated = true;
  _stack.push_front(_estimate);
}

template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
void VariableGeneric_<PerturbationDim_, EstimateType_>::pop()
{
  assert(!_stack.empty() && "VariableGeneric_::pop|ERROR, invalid stack");
  _updated  = true;
  _estimate = _stack.front();
  _stack.pop_front();
}

template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
void VariableGeneric_<PerturbationDim_, EstimateType_>::discardTop()
{
  assert(!_stack.empty() && "VariableGeneric_::discardTop|ERROR, invalid stack");
  _stack.pop_front();
}

template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
void Variable_<PerturbationDim_, EstimateType_>::serialize(
  ObjectData & odata,
  IdContext & context)
{
  Identifiable::serialize(odata, context);
  odata.setInt("graph_id", this->graphId());
  odata.setInt("status", this->status());

  // -std=c++Giorgio
  //    using EstimateType = typeof(this->estimate());
  //    odata.setEigen<EstimateType>("estimate", this->_estimate);

  // -std=c++14
  odata.setEigen<decltype(this->_estimate)>("estimate", this->_estimate);

  // old serialization of eigen data
  //  const int e_rows = this->_estimate.matrix().rows();
  //  const int e_cols = this->_estimate.matrix().cols();
  //  ArrayData* adata = new ArrayData;
  //  for (int r = 0; r < e_rows; ++r) {
  //    for (int c = 0; c < e_cols; ++c) {
  //      adata->add(this->_estimate.matrix()(r, c));
  //    }
  //  }
  //  odata.setField("estimate", adata);
}

template <int PerturbationDim_, template <typename Scalar_> typename EstimateType_>
void Variable_<PerturbationDim_, EstimateType_>::deserialize(
  ObjectData & odata,
  IdContext & context)
{
  Identifiable::deserialize(odata, context);
  this->setGraphId(odata.getInt("graph_id"));
  VariableBase::Status st;
  int s = odata.getInt("status");
  switch (s) {
    case VariableBase::Status::Active:
      st = VariableBase::Status::Active;
      break;
    case VariableBase::Status::NonActive:
      st = VariableBase::Status::NonActive;
      break;
    case VariableBase::Status::Fixed:
      st = VariableBase::Status::Fixed;
      break;
    default:
      throw std::runtime_error("Variable_::deserialize|ERROR, invalid status value in file");
  }
  this->setStatus(st);

  // -std=c++Giorgio
  //    using EstimateType = typeof(this->_estimate);
  //    EstimateType est;
  //    est = odata.getEigen<EstimateType>("estimate");

  // -std=c++14
  auto est = odata.getEigen<decltype(this->_estimate)>("estimate");

  this->setEstimate(est);

  // old serialization of eigen data
  //    ArrayData* adata = dynamic_cast<ArrayData*>(odata.getField("estimate"));
  //    int k            = 0;
  //    const int e_rows = this->_estimate.matrix().rows();
  //    const int e_cols = this->_estimate.matrix().cols();
  //    for (int r = 0; r < e_rows; ++r) {
  //      for (int c = 0; c < e_cols; ++c, ++k) {
  //        this->_estimate.matrix()(r, c) = (*adata)[k].getFloat();
  //      }
  //    }
  //    this->setEstimate(this->_estimate);
}

/*! Associative container, the key is the graph id while the value is pointer to the corresponding
  * variable */
using IdVariablePtrContainer = AbstractMapView_<VariableBase::Id, VariableBase*>;
using IdVariablePair         = std::pair<VariableBase::Id, VariableBase*>;

} // namespace srrg2_solver
