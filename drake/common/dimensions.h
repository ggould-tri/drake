/// @file This file provides a "dimensioned value" facility to Drake, similar
/// in concept to systems like `boost::units` but less capable.
///
/// The Boost units library is unfortunately unsuitable, as (1) Matlab is
/// incompatible with any use of boost, and (2) Eigen is incompatible with
/// compile-time dimension checking.  Instead, this module provides some
/// limited runtime dimension checking.

#include <Eigen/Core>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_export.h"

namespace drake {

namespace {
// The basic dimensions available; this list can and should be expanded as
// more dimensions are added.
enum BaseDimension {
  kDistance,  //< In units of meters.
  kTime,  //< In units of seconds.
  kMass,  //< In units of kilograms.
  kNumDimensions
};

constexpr BaseDimension all_dimensions[kNumDimensions] = {
  kDistance, kTime, kMass};
}  // anon namespace

/// A Dimensioned<T> is a wrapper around a T that adds dimensional metadata.
/// This prevents, for instance, adding a force to a torque, or comparing a
/// distance and a power.  If you try to do so, an error will be thrown at
/// runtime.
///
/// Dimensioned quantities are constructed using unit-named static methods, eg
/// `meters(2.)` to denote 2m.  All T are assumed to be in MKS units.
///
/// Because a large fraction of our code is not unitized, certain unsafe
/// operations are allowed transitionally.  Undimensioned quantities are
/// allowed to be added and subtracted to dimensioned quantities, for
/// instance.
///
/// Because `Dimensioned<T>` is meant to be a drop-in replacement for any `T`,
/// it uses the default constructors.
///
/// Many of these methods are declared inline and written in the header
/// because in practical use cases once inlined they can be
/// constant-propagated heavily.
template<typename T>
class Dimensioned {
 public:
  //@{
  /// Unit constructors.
  static inline Dimensioned<T> meters(T quantity) {
    const int8_t dims[kNumDimensions] = {1, 0, 0};
    return Dimensioned(quantity, dims);
  }
  const static Dimensioned<T> meter() { return meters(1); }
  static inline Dimensioned<T> kilograms(T quantity) {
    const int8_t dims[kNumDimensions] = {0, 0, 1};
    return Dimensioned(quantity, dims);
  }
  const static Dimensioned<T> kilogram() { return kilograms(1); }
  static inline Dimensioned<T> seconds(T quantity) {
    const int8_t dims[kNumDimensions] = {0, 1, 0};
    return Dimensioned(quantity, dims);
  }
  const static Dimensioned<T> second() { return seconds(1); }
  static inline Dimensioned<T> newtons(T quantity) {
    const int8_t dims[kNumDimensions] = {1, -2, 1};
    return Dimensioned(quantity, dims);
  }
  const static Dimensioned<T> newton() { return newtons(1); }
  //@}

  bool same_dimension(const Dimensioned<T> other) const;

  inline Dimensioned<T> operator*(const Dimensioned<T>& rhs) const {
    int8_t dims[kNumDimensions];
    for (const BaseDimension& i : all_dimensions) {
      dims[i] = dimension_exponents_[i] + rhs.dimension_exponents_[i];
    }
    return Dimensioned<T>(value_ * rhs.value_, dims);
  }

  inline Dimensioned<T> operator/(const Dimensioned<T>& rhs) const {
    int8_t dims[kNumDimensions];
    for (const BaseDimension& i : all_dimensions) {
      dims[i] = dimension_exponents_[i] - rhs.dimension_exponents_[i];
    }
    return Dimensioned<T>(value_ / rhs.value_, dims);
  }

  inline Dimensioned<T> operator+(const Dimensioned<T>& rhs) const {
    DRAKE_DEMAND(same_dimension(rhs));
    return Dimensioned<T>(value_ + rhs.value_, dimension_exponents_);
  }

  inline Dimensioned<T> operator-(const Dimensioned<T>& rhs) const {
    DRAKE_DEMAND(same_dimension(rhs));
    return Dimensioned<T>(value_ - rhs.value_, dimension_exponents_);
  }

  inline bool operator==(const Dimensioned<T>& rhs) const {
    DRAKE_DEMAND(same_dimension(rhs));
    return value_ == rhs.value_;
  }

  inline bool operator<(const Dimensioned<T>& rhs) const {
    DRAKE_DEMAND(same_dimension(rhs));
    return value_ < rhs.value_;
  }

  // Defined in terms of the previous operators to minimize logic duplication:
  inline bool operator>(const Dimensioned<T>& rhs) const {
    return !(*this < rhs) && !(*this == rhs);
  }
  inline bool operator>=(const Dimensioned<T>& rhs) const {
    return !(*this < rhs);
  }
  inline bool operator<=(const Dimensioned<T>& rhs) const {
    return (*this < rhs) || (*this == rhs);
  }
  inline bool operator!=(const Dimensioned<T>& rhs) const {
    return !(*this == rhs);
  }

  friend Dimensioned<T> operator*(const T& scalar, const Dimensioned<T>& rhs) {
    return Dimensioned<T>(rhs.value_ * scalar, rhs.dimension_exponents_);
  }

  friend Dimensioned<T> operator*(const Dimensioned<T>& lhs, const T& scalar) {
    return Dimensioned<T>(lhs.value_ * scalar, lhs.dimension_exponents_);
  }

  friend Dimensioned<T> operator/(const T& scalar, const Dimensioned<T>& rhs) {
    int8_t dims[kNumDimensions];
    for (const BaseDimension& i : all_dimensions) {
      dims[i] = -rhs.dimension_exponents_[i];
    }
    return Dimensioned<T>(rhs.value_ / scalar, dims);
  }

  friend Dimensioned<T> operator/(const Dimensioned<T>& lhs, const T& scalar) {
    return Dimensioned<T>(lhs.value_ / scalar, lhs.dimension_exponents_);
  }

 private:
  Dimensioned(const T& value, const int8_t dimension_exponents[kNumDimensions]);

  T value_;  //< The quantity of the given dimension present, in MKS units.

  // Denotes the exponent of each dimension in the dimension of this quantity.
  // For instance, a quantity of energy would have `dimension_exponents_` of:
  // [ (kDistance) 1, (kTime) -2, (kMass) 1].
  int8_t dimension_exponents_[kNumDimensions];
};

}  // drake


namespace Eigen {

/// Inform Eigen of the type traits that allow this type to participate in
/// matrices and vectors.  This is defined for any T that is itself an Eigen
/// scalartype.
///
/// The documentation of how Eigen type traits work is at:
///  http://eigen.tuxfamily.org/dox/TopicCustomizingEigen.html#CustomScalarType
template<typename T>
struct NumTraits<drake::Dimensioned<T>>
 : NumTraits<T> {
  const static int dimension_overhead = 1;

  typedef drake::Dimensioned<T> Real;
  typedef drake::Dimensioned<T> NonInteger;
  typedef drake::Dimensioned<T> Nested;
  enum {
    IsComplex = 0,
    IsInteger = 0,
    IsSigned = 1,
    RequireInitialization = 1,
    ReadCost = NumTraits<T>::ReadCost + dimension_overhead,
    AddCost = NumTraits<T>::AddCost + dimension_overhead,
    MulCost = NumTraits<T>::MulCost + dimension_overhead
  };
};

}
