#include "drake/util/TrigPoly.h"

template<typename CoefficientType>
TrigPoly<CoefficientType>::TrigPoly(const CoefficientType& scalar)
    : poly(scalar) {}

template<typename CoefficientType>
TrigPoly<CoefficientType>::TrigPoly(
    const PolyType& p, const SinCosMap& _sin_cos_map)
    : poly(p), sin_cos_map(_sin_cos_map) {
  // The provided _sin_cos_map might have extraneous entries; clip them.
  std::set<VarType> vars_in_use = p.getVariables();
  for (const auto& sin_cos_entry : _sin_cos_map) {
    if (!vars_in_use.count(sin_cos_entry.first)) {
      sin_cos_map.erase(sin_cos_entry.first);
    }
  }
}

template class DRAKEPOLYNOMIAL_EXPORT TrigPoly<double>;
