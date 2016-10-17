#include "drake/common/dimensions.h"
#include "drake/common/polynomial.h"

namespace drake {

template class DRAKE_EXPORT Dimensioned<double> ;
template class DRAKE_EXPORT Dimensioned<Polynomial<double>> ;

template<typename T>
Dimensioned<T>::Dimensioned()
    : value_(-99) {
  for (const BaseDimension& i : all_dimensions) {
    dimension_exponents_[i] = -99;  // Arbitrary useless value.
  }
}

template<typename T>
Dimensioned<T>::Dimensioned(const T& value)
    : value_(value) {
  for (const BaseDimension& i : all_dimensions) {
    dimension_exponents_[i] = -99;  // Arbitrary useless value.
  }
}

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

template<typename T>
std::string Dimensioned<T>::dimension_str() const {
  std::string numerator = "";
  std::string denominator = "";
  if (dimension_exponents_[kMass] > 0) {
    numerator += "kg";
    if (dimension_exponents_[kMass] > 1) {
      numerator += "^" + dimension_exponents_[kMass];
    }
  } else if (dimension_exponents_[kMass] < 0) {
    denominator += "kg";
    if (dimension_exponents_[kMass] < -1) {
      denominator += "^" + -dimension_exponents_[kMass];
    }
  }
  if (dimension_exponents_[kDistance] > 0) {
    numerator += "m";
    if (dimension_exponents_[kDistance] > 1) {
      numerator += "^" + dimension_exponents_[kDistance];
    }
  } else if (dimension_exponents_[kDistance] < 0) {
    denominator += "m";
    if (dimension_exponents_[kDistance] < -1) {
      denominator += "^" + -dimension_exponents_[kDistance];
    }
  }
  if (dimension_exponents_[kTime] > 0) {
    numerator += "s";
    if (dimension_exponents_[kTime] > 1) {
      numerator += "^" + dimension_exponents_[kTime];
    }
  } else if (dimension_exponents_[kTime] < 0) {
    denominator += "s";
    if (dimension_exponents_[kTime] < -1) {
      denominator += "^" + -dimension_exponents_[kTime];
    }
  }
  if (numerator != "" && denominator != "") {
    return numerator + "/" + denominator;
  } else if (numerator != "") {
    return numerator;
  } else if (denominator != "") {
    return "1/" + denominator;
  } else {
    return "";
  }
}

template<typename T>
Dimensioned<T> Dimensioned<T>::operator/(
    const Dimensioned<T>& rhs) const {
  int8_t dims[kNumDimensions];
  for (const BaseDimension& i : all_dimensions) {
    dims[i] = dimension_exponents_[i] - rhs.dimension_exponents_[i];
  }
  return Dimensioned<T>(value_ / rhs.value_, dims);
}

template<>
Dimensioned<Polynomial<double>> Dimensioned<Polynomial<double>>::operator/(
    const Dimensioned<Polynomial<double>>& rhs) const {
  DRAKE_ABORT();
}

}
