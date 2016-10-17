#include "drake/common/dimensions.h"
#include "drake/common/drake_assert.h"

namespace drake {

template<typename T>
Dimensioned<T>::Dimensioned(
    const T& value,
    const int8_t dimension_exponents[kNumDimensions])
  : value_(value) {
  for (const BaseDimension& i : all_dimensions) {
    dimension_exponents_[i] = dimension_exponents[i];
  }
}

template<typename T>
bool Dimensioned<T>::same_dimension(const Dimensioned<T> other) const {
  for (const BaseDimension& i : all_dimensions) {
    if (dimension_exponents_[i] != other.dimension_exponents_[i]) {
      return false;
    }
  }
  return true;
}

template class DRAKE_EXPORT Dimensioned<double> ;

}
