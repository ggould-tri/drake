#include "drake/solvers/SystemIdentification.h"

#include <algorithm>
#include "drake/solvers/Optimization.h"

namespace drake {
namespace solvers {

template<typename T>
std::set<typename SystemIdentification<T>::MonomialType>
SystemIdentification<T>::GetAllCombinationsOfVars(
    const std::vector<PolyType>& polys,
    const std::set<VarType>& vars_of_interest) {
  std::set<MonomialType> interest_monomials;
  for (const PolyType& poly : polys) {
    for (const MonomialType& monomial : poly.getMonomials()) {
      typename PolyType::Monomial interest_monomial;
      interest_monomial.coefficient = 1;
      for (const TermType& term : monomial.terms) {
        if (vars_of_interest.count(term.var)) {
          interest_monomial.terms.push_back(term);
        }
      }
      interest_monomials.insert(interest_monomial);
    }
  }
  return interest_monomials;
}

template<typename T>
bool SystemIdentification<T>::MonomialMatches(
    const MonomialType& haystack,
    const MonomialType& needle,
    const std::set<VarType>& vars_of_interest) {
  const MonomialType residue = haystack.factor(needle);
  if (residue.coefficient == 0) {
    return false;
  }
  for (const VarType& var : vars_of_interest) {
    if (residue.getDegreeOf(var) > 0) {
      return false;
    }
  }
  return true;
}

template<typename T>
std::pair<T, typename SystemIdentification<T>::PolyType>
SystemIdentification<T>::NormalizePolynomial(const PolyType& poly) {
  std::vector<MonomialType> monomials = poly.getMonomials();
  const T min_coefficient = std::min_element(
      monomials.begin(), monomials.end(),
      [&](const MonomialType& l, const MonomialType& r){
        return l.coefficient < r.coefficient; })->coefficient;
  for (MonomialType& monomial : monomials) {
    monomial.coefficient /= min_coefficient;
  }
  return std::make_pair(min_coefficient, PolyType(monomials.begin(),
                                                  monomials.end()));
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomial(
    const PolyType& poly,
    const std::set<VarType>& vars_of_interest) {
  // Just dispatch to the set version.
  const std::vector<Polynomial<T>> polys = {poly};
  return SystemIdentification<T>::GetLumpedParametersFromPolynomials(
      polys, vars_of_interest);
}

template<typename T>
typename SystemIdentification<T>::LumpingMapType
SystemIdentification<T>::GetLumpedParametersFromPolynomials(
    const std::vector<PolyType>& polys,
    const std::set<VarType>& vars_of_interest) {
  // Before we begin, check that we can reserve some names (VarType values)
  // for our lumped parameters.
  std::set<VarType> all_vars;
  for (const PolyType& poly : polys) {
    const auto& poly_vars = poly.getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }
  const VarType reservation_start = Polynomiald("lump", 1).getSimpleVariable();
  const VarType reservation_end = Polynomiald("lump", 1000).getSimpleVariable();
  for (const VarType& var : all_vars) {
    if ((var >= reservation_start) && (var <= reservation_end)) {
      throw std::runtime_error(
          "Lumped parameters failed because variable name already in use");
    }
  }

  // Extract every combination of the vars_of_interest.
  const std::set<typename PolyType::Monomial> interest_monomials =
      GetAllCombinationsOfVars(polys, vars_of_interest);

  // For each of those combinations, find the corresponding polynomials of
  // parameter (ie, non of-interest) variables in each polynomial.
  std::set<PolyType> lumped_parameters;
  for (const MonomialType& interest_monomial : interest_monomials) {
    for (const PolyType& poly : polys) {
      std::vector<MonomialType> lumped_parameter;
      for (const MonomialType& monomial : poly.getMonomials()) {
        if (MonomialMatches(monomial, interest_monomial, vars_of_interest)) {
          lumped_parameter.push_back(monomial.factor(interest_monomial));
        }
      }
      if (!lumped_parameter.size()) { continue; }
      // Factor out any coefficients, so that 'a' and '2*a' are not both
      // considered lumped parameters.
      PolyType lumped_parameter_polynomial(lumped_parameter.begin(),
                                           lumped_parameter.end());
      PolyType normalized =
          NormalizePolynomial(lumped_parameter_polynomial).second;
      lumped_parameters.insert(normalized);
    }
  }

  // For each such parameter polynomial, create a lumped parameter.
  int lump_index = 1;
  typename SystemIdentification<T>::LumpingMapType lumping_map;

  for (const PolyType& lump : lumped_parameters) {
    VarType lump_var = PolyType("lump", lump_index).getSimpleVariable();
    lumping_map[lump] = lump_var;
    lump_index++;
  }

  return lumping_map;
}

template<typename T>
typename SystemIdentification<T>::PolyType
SystemIdentification<T>::RewritePolynomialWithLumpedParameters(
    const PolyType& poly,
    const LumpingMapType& lumped_parameters) {
  // Reconstruct vars_of_interest, the variables in poly that are not
  // mentioned by the lumped_parameters.
  std::set<VarType> vars_of_interest = poly.getVariables();
  for (auto lump_name_pair : lumped_parameters) {
    std::set<VarType> parameters_in_lump = lump_name_pair.first.getVariables();
    for (const VarType& var : parameters_in_lump) {
      vars_of_interest.erase(var);
    }
  }

  // Loop over the combinations of the variables-of-interest, constructing the
  // polynomial of parameters that multiply by each combination; if that
  // polynomial is a lumped variable, substitute in a new monomial of the
  // lumped variable times the combination instead.
  std::set<typename PolyType::Monomial> interest_monomials =
      GetAllCombinationsOfVars({poly}, vars_of_interest);
  std::vector<MonomialType> working_monomials = poly.getMonomials();
  for (const MonomialType& interest_monomial : interest_monomials) {
    std::vector<MonomialType> new_working_monomials;
    std::vector<MonomialType> factor_monomials;
    for (const MonomialType& working_monomial : working_monomials) {
      if (MonomialMatches(working_monomial, interest_monomial,
                          vars_of_interest)) {
        // This monomial matches our interest monomial; we will factor it by
        // the interest monomial and add the resulting monomial of parameters
        // to our factor list.
        factor_monomials.push_back(working_monomial.factor(interest_monomial));
      } else {
        // This monomial does not match our interest monomial; copy it
        // unchanged.
        new_working_monomials.push_back(working_monomial);
      }
    }
    const PolyType factor_polynomial(factor_monomials.begin(),
                               factor_monomials.end());
    const auto& normalization = NormalizePolynomial(factor_polynomial);
    const T coefficient = normalization.first;
    const PolyType& normalized = normalization.second;

    if (!lumped_parameters.count(normalized)) {
      // Factoring out this combination yielded a parameter polynomial that
      // does not correspond to a lumped variable.  Ignore it.
      continue;
    }

    // We have a lumped parameter, so construct a new monomial and replace the
    // working monomials list.
    TermType lump_term;
    lump_term.var = lumped_parameters.at(normalized);
    lump_term.power = 1;
    MonomialType lumped_monomial;
    lumped_monomial.terms = interest_monomial.terms;
    lumped_monomial.terms.push_back(lump_term);
    lumped_monomial.coefficient = coefficient;
    new_working_monomials.push_back(lumped_monomial);
    working_monomials = new_working_monomials;
  }

  return PolyType(working_monomials.begin(), working_monomials.end());
}

template<typename T>
typename SystemIdentification<T>::ValueMappingType
SystemIdentification<T>::EstimateLumpedParameters(
    const PolyType& expr,
    const std::vector<const ValueMappingType>& empirical_variable_values,
    const std::vector<CoefficientType>& empirical_expr_values) {
  // Gather up our empirical variables and parameters.
  const std::set<VarType> all_vars = expr.getVariabls();
  std::set<VarType> empirical_vars;
  for (const ValueMappingType& mapping : empirical_variable_values) {
    for (const auto& map_pair : mapping) {
      empirical_vars.insert(map_pair.first);
    }
  }
  std::vector<VarType> parameter_vars;  // std::vector to ensure stable order.
  std::set_difference(all_vars.begin(), all_vars.end(),
                      empirical_vars.begin(), empirical_vars.end(),
                      std::inserter(parameter_vars.end()));

  ... linearize?
  
  // Build an optimization problem with slots for the parameter variables.
  Drake::OptimizationProblem problem;
  problem.AddContinuousVariables(parameter_vars.size());
  

  // Build a quadratic cost for each observation.
  

  // Fire off the solver.
  

  // Extract the parameter values from the optimization problem's slots into
  // the return mapping.
  

  return ValueMappingType();
}

};
};

template class DRAKEPOLYNOMIAL_EXPORT
drake::solvers::SystemIdentification<double>;
