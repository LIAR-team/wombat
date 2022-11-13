// Copyright 2018-2022, Giorgio Grisetti, Mirco Colosi, Dominik Schlegel,
// Bartolomeo Della Corte, Irvin Aloise, Federico Nardi, Tiziano Guadagnino

#pragma once

#include <math.h>

#include <Eigen/Geometry>

namespace srrg2_core
{

namespace ad {

  inline float acos(const float & val_)
  {
    return acosf(val_);
  }


  inline float sqrt(const float & val_)
  {
    return sqrtf(val_);
  }

  inline float cos(const float & angle_)
  {
    return cosf(angle_);
  }

  inline float sin(const float & angle_)
  {
    return sinf(angle_);
  }

  inline float normalizeAngle(const float & angle_)
  {
    return atan2(sin(angle_), cos(angle_));
  }

  inline double normalizeAngle(const double & angle_)
  {
    return atan2(sin(angle_), cos(angle_));
  }

  /**dual value, stores the elements of autodiff
      it reperesent a pair
      u, u'
      and defines all common operators	      
      +,-,*,/, and so on

      DualValue is parametiric w.r.t the base type (float or double)
  */

  template <typename T>
  class DualValue_
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef T BaseType;
    T value;
    T derivative;

    inline operator T() const
    {
      return value;
    }

    DualValue_()
    {
      value = 0, derivative = 0;
    }

    DualValue_(const T& v)
    {
      value = v;
      derivative = 0;
    }

    DualValue_(const T& v, const T& d)
    {
      value = v;
      derivative = d;
    }

    inline DualValue_& operator=(const T& v)
    {
      value = v;
      derivative = 0;
      return *this;
    }

    inline DualValue_& operator+=(const DualValue_& op)
    {
      value += op.value;
      derivative += op.derivative;
      return *this;
    }

    inline DualValue_& operator-=(const DualValue_& op)
    {
      value -= op.value;
      derivative -= op.derivative;
      return *this;
    }

    inline DualValue_& operator*=(const DualValue_& op)
    {
      value *= op.value;
      derivative = derivative * op.value + value * op.derivative;
      return *this;
    }

    inline DualValue_& operator/=(const DualValue_& op)
    {
      value /= op.value;
      derivative = (derivative * op.value - value * op.derivative) / (op.value * op.value);
      return *this;
    }

    inline bool operator>(const DualValue_& v) const
    {
      return value > v.value;
    }

    inline bool operator>=(const DualValue_& v) const
    {
      return value >= v.value;
    }

    inline bool operator==(const DualValue_& v) const
    {
      return value == v.value;
    }

    inline bool operator!=(const DualValue_& v) const
    {
      return value != v.value;
    }

    inline bool operator<(const DualValue_& v) const
    {
      return value < v.value;
    }

    inline bool operator<=(const DualValue_& v) const
    {
      return value <= v.value;
    }

  };

  template <typename T>
  inline std::ostream& operator<<(std::ostream& os, const DualValue_<T>& v)
  {
    os << "(v: " << v.value << " " << " d: " << v.derivative << ")";
    return os;
  }

  typedef DualValue_<float> DualValuef;
  typedef DualValue_<double> DualValued;

  template <typename T>
  inline DualValue_<T> abs(const DualValue_<T>& op)
  {
    return DualValue_<T>(std::abs(op.value), (op.value > 0 ? op.derivative : -op.derivative));
  }

  template <typename T>
  inline DualValue_<T> operator+(const DualValue_<T>& op)
  {
    return op;
  }

  template <typename T>
  inline DualValue_<T> operator-(const DualValue_<T>& op2)
  {
    return DualValue_<T>(-op2.value, -op2.derivative);
  }

  template <typename T>
  inline DualValue_<T> operator+(const DualValue_<T>& op1, const DualValue_<T>& op2)
  {
    return DualValue_<T>(op1.value + op2.value, op1.derivative + op2.derivative);
  }

  template <typename T>
  inline DualValue_<T> operator-(const DualValue_<T>& op1, const DualValue_<T>& op2)
  {
    return DualValue_<T>(op1.value - op2.value, op1.derivative - op2.derivative);
  }

  template <typename T>
  inline DualValue_<T> operator*(const DualValue_<T>& op1, const DualValue_<T>& op2)
  {
    return DualValue_<T>(op1.value * op2.value, op1.derivative * op2.value + op1.value * op2.derivative);
  }

  template <typename T>
  inline DualValue_<T> operator/(const DualValue_<T>& op1, const DualValue_<T>& op2)
  {
    return DualValue_<T>(op1.value / op2.value, (op1.derivative * op2.value - op1.value * op2.derivative) / (op2.value * op2.value));
  }

  template <typename T>
  inline DualValue_<T> sin(const DualValue_<T>& op)
  {
    return DualValue_<T>(sin(op.value), cos(op.value) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> asin(const DualValue_<T>& op)
  {
    return DualValue_<T>(asin(op.value), 1./(sqrt(1 - op.value*op.value)) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> cos(const DualValue_<T>& op)
  {
    return DualValue_<T>(cos(op.value), -sin(op.value) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> acos(const DualValue_<T>& op)
  {
    return DualValue_<T>(acos(op.value), -1./(sqrt(1 - op.value*op.value)) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> log(const DualValue_<T>& op)
  {
    return DualValue_<T>(log(op.value), 1. / fabs(op.value) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> exp(const DualValue_<T>& op)
  {
    return DualValue_<T>(exp(op.value), exp(op.value) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> sqrt(const DualValue_<T>& op)
  {
    return DualValue_<T>(sqrt(op.value), 0.5 / sqrt(op.value) * op.derivative);
  }

  template <typename T>
  inline DualValue_<T> atan2(const DualValue_<T>& op1, const DualValue_<T>& op2)
  {
    return DualValue_<T>(atan2(op1.value, op2.value),
                          1. / (1 + pow(op1.value / op2.value, 2))
                          * (op1.derivative * op2.value - op1.value * op2.derivative)
                          / (op2.value * op2.value));
  }

  template <typename DestType_, typename SrcType_>
  void convertMatrix(DestType_& dest, const SrcType_& src)
  {
    if (src.cols() != dest.cols()) {
      throw std::runtime_error("cols size mismatch");
    }
    if (src.rows() != dest.rows()) {
      throw std::runtime_error("rows size mismatch");
    }
    typedef typename DestType_::Scalar DestScalar;
    for (int c = 0; c < src.cols(); ++c) {
      for (int r = 0; r < src.rows(); ++r) {
        dest(r, c) = DestScalar(src(r, c));
      }
    }
  }
}

using DualValuef = ad::DualValue_<float>;
using DualValued = ad::DualValue_<double>;

}
