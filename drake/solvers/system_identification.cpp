#include "drake/solvers/system_identification.h"

#include <algorithm>
#include <cmath>

#include "drake/common/drake_assert.h"
#include "drake/solvers/Optimization.h"
#include "drake/util/Polynomial.h"
#include "drake/util/TrigPoly.h"

namespace drake {
namespace solvers {

template<typename ExprType>
std::set<typename SystemIdentification<ExprType>::MonomialType>
SystemIdentification<ExprType>::GetAllCombinationsOfVars(
    const std::vector<ExprType>& polys,
    const std::set<VarType>& vars) {
  std::set<MonomialType> result_monomials;
  for (const ExprType& poly : polys) {
    for (const MonomialType& monomial : poly.getMonomials()) {
      MonomialType monomial_of_vars;
      monomial_of_vars.coefficient = 1;
      // For each term in the monomial, iff that term's variable is in
      // vars then multiply it in.
      for (const TermType& term : monomial.terms) {
        if (vars.count(term.var)) {
          monomial_of_vars.terms.push_back(term);
        }
      }
      result_monomials.insert(monomial_of_vars);
    }
  }
  return result_monomials;
}

template<typename ExprType>
bool SystemIdentification<ExprType>::MonomialMatches(
    const MonomialType& haystack,
    const MonomialType& needle,
    const std::set<VarType>& active_vars) {
  // By factoring the needle out of the haystack, we either get a failure (in
  // which case return false) or a residue monomial (the parts of haystack not
  // in needle).  If the resuidue contains an active variable, return false.
  // Otherwise we meet the criteria and return true.
  const MonomialType residue = haystack.factor(needle);
  if (residue.coefficient == 0) {
    return false;
  }
  for (const VarType& var : active_vars) {
    if (residue.getDegreeOf(var) > 0) {
      return false;
    }
  }
  return true;
}

template<typename ExprType>
std::pair<typename ExprType::CoefficientType, ExprType>
SystemIdentification<ExprType>::CanonicalizePolynomial(const ExprType& poly) {
  std::vector<MonomialType> monomials = poly.getMonomials();
  const CoefficientType min_coefficient = std::min_element(
      monomials.begin(), monomials.end(),
      [&](const MonomialType& l, const MonomialType& r){
        return l.coefficient < r.coefficient; })->coefficient;
  for (MonomialType& monomial : monomials) {
    monomial.coefficient /= min_coefficient;
  }
  return std::make_pair(min_coefficient, ExprType(monomials.begin(),
                                                  monomials.end()));
}

template<typename ExprType>
typename SystemIdentification<ExprType>::LumpingMapType
SystemIdentification<ExprType>::GetLumpedParametersFromPolynomial(
    const ExprType& poly,
    const std::set<VarType>& parameters) {
  // Just dispatch to the set version.
  const std::vector<ExprType> polys = {poly};
  return SystemIdentification<ExprType>::GetLumpedParametersFromPolynomials(
      polys, parameters);
}

template<typename ExprType>
typename SystemIdentification<ExprType>::LumpingMapType
SystemIdentification<ExprType>::GetLumpedParametersFromPolynomials(
    const std::vector<ExprType>& polys,
    const std::set<VarType>& parameters) {
  // Before we begin, find all the VarTypes in use so that we can create
  // unique ones for our lumped parameters, and find the list of active
  // variables..
  std::set<VarType> all_vars;
  for (const ExprType& poly : polys) {
    const auto& poly_vars = poly.getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }
  std::set<VarType> active_vars = all_vars;
  for (const VarType& parameter : parameters) {
    active_vars.erase(parameter);
  }

  // Extract every combination of the active_vars.
  const std::set<MonomialType> active_var_monomials =
      GetAllCombinationsOfVars(polys, active_vars);

  // For each of those combinations, find the corresponding polynomials of
  // parameter variables in each polynomial.
  std::set<ExprType> lumped_parameters;
  for (const MonomialType& active_var_monomial : active_var_monomials) {
    for (const ExprType& poly : polys) {
      std::vector<MonomialType> lumped_parameter;
      for (const MonomialType& monomial : poly.getMonomials()) {
        // NOTE: This may be a performance hotspot if this method is called in
        // a tight loop, due to the nested for loops here and in
        // MonomialMatches and its callees.  If so it can be sped up via loop
        // reordering and intermediate maps at some cost to readability.
        if (MonomialMatches(monomial, active_var_monomial, active_vars)) {
          lumped_parameter.push_back(monomial.factor(active_var_monomial));
        }
      }
      if (!lumped_parameter.size()) { continue; }
      // Factor out any coefficients, so that 'a' and '2*a' are not both
      // considered lumped parameters.
      Polynomial<CoefficientType> lumped_parameter_polynomial(
          lumped_parameter.begin(),
          lumped_parameter.end());
      Polynomial<CoefficientType> normalized =
          CanonicalizePolynomial(lumped_parameter_polynomial).second;

      // Build the lumped parameters as appropriate to our specialized type.
      auto lumped_params_as_polynomials =
          dynamic_cast<std::set<Polynomial<CoefficientType>>*>(
              &lumped_parameters);
      if (lumped_params_as_polynomials) {
        lumped_params_as_polynomials->insert(normalized);
      }
      auto lumped_params_as_trigpolys =
          dynamic_cast<std::set<TrigPoly<CoefficientType>>*>(
              &lumped_parameters);
      if (lumped_params_as_trigpolys) {
        TrigPoly<CoefficientType> normalized_trigpoly(
            normalized, poly.getSinCosMap());
        lumped_params_as_trigpolys->insert(normalized_trigpoly);
      }
    }
  }

  // For each such parameter polynomial, create a lumped parameter with a
  // unique VarType id.
  LumpingMapType lumping_map;
  for (const ExprType& lump : lumped_parameters) {
    VarType lump_var = CreateUnusedVar("lump", all_vars);
    lumping_map[lump] = lump_var;
    all_vars.insert(lump_var);
  }

  return lumping_map;
}

template<typename ExprType>
typename SystemIdentification<ExprType>::VarType
SystemIdentification<ExprType>::CreateUnusedVar(
    const std::string& prefix,
    const std::set<VarType>& vars_in_use) {
  int lump_index = 1;
  while (true) {
    VarType lump_var = ExprType(prefix, lump_index).getSimpleVariable();
    lump_index++;
    if (!vars_in_use.count(lump_var)) {
      return lump_var;
    }
  }  // Loop termination: If every id is already used, ExprType() will throw.
}


template<typename ExprType>
ExprType SystemIdentification<ExprType>::RewritePolynomialWithLumpedParameters(
    const ExprType& poly,
    const LumpingMapType& lumped_parameters) {
  // Reconstruct active_vars, the variables in poly that are not
  // mentioned by the lumped_parameters.
  std::set<VarType> active_vars = poly.getVariables();
  for (const auto& lump_name_pair : lumped_parameters) {
    std::set<VarType> parameters_in_lump = lump_name_pair.first.getVariables();
    for (const VarType& var : parameters_in_lump) {
      active_vars.erase(var);
    }
  }

  // Loop over the combinations of the active variables, constructing the
  // polynomial of parameters that multiply by each combination; if that
  // polynomial is a lumped variable, substitute in a new monomial of the
  // lumped variable times the combination instead.
  std::set<MonomialType> active_var_monomials =
      GetAllCombinationsOfVars({poly}, active_vars);
  std::vector<MonomialType> working_monomials = poly.getMonomials();
  for (const MonomialType& active_var_monomial : active_var_monomials) {
    // Because we must iterate over working_monomials, we cannot alter it in
    // place.  Instead we build up two lists in parallel: The updated value of
    // working_monomials (new_working_monomials) unchanged by rewriting and
    // the monomials of parameter variables that might form the polynomial
    // of a lumped parameter.
    //
    // If (and only if) the polynomial of factored monomials matches a lumped
    // parameter, we construct a new working_monomials list from the
    // new_working_monomials list plus a lumped-parameter term.
    std::vector<MonomialType> new_working_monomials;
    std::vector<MonomialType> factor_monomials;
    for (const MonomialType& working_monomial : working_monomials) {
      if (MonomialMatches(working_monomial, active_var_monomial,
                          active_vars)) {
        // This monomial matches our active vars monomial; we will factor it
        // by the active vars monomial and add the resulting monomial of
        // parameters to our factor list.
        factor_monomials.push_back(
            working_monomial.factor(active_var_monomial));
      } else {
        // This monomial does not match our active vars monomial; copy it
        // unchanged.
        new_working_monomials.push_back(working_monomial);
      }
    }
    const ExprType factor_polynomial(factor_monomials.begin(),
                                     factor_monomials.end());
    const auto& canonicalization = CanonicalizePolynomial(factor_polynomial);
    const CoefficientType coefficient = canonicalization.first;
    const ExprType& canonicalized = canonicalization.second;

    if (!lumped_parameters.count(canonicalized)) {
      // Factoring out this combination yielded a parameter polynomial that
      // does not correspond to a lumped variable.  Ignore it, because we
      // cannot rewrite it correctly.
      //
      // This can happen if `poly` was not one of the polynomials used to
      // construct `lumped_parameters`.
      continue;
    }

    // We have a lumped parameter, so construct a new monomial and replace the
    // working monomials list.
    TermType lump_term;
    lump_term.var = lumped_parameters.at(canonicalized);
    lump_term.power = 1;
    MonomialType lumped_monomial;
    lumped_monomial.terms = active_var_monomial.terms;
    lumped_monomial.terms.push_back(lump_term);
    lumped_monomial.coefficient = coefficient;
    new_working_monomials.push_back(lumped_monomial);
    working_monomials = new_working_monomials;
  }

  return ExprType(working_monomials.begin(), working_monomials.end());
}

template<typename ExprType>
std::pair<typename SystemIdentification<ExprType>::PartialEvalType,
          typename ExprType::CoefficientType>
SystemIdentification<ExprType>::EstimateParameters(
    const VectorXExpr& polys,
    const std::vector<PartialEvalType>& active_var_values) {
  DRAKE_ASSERT(active_var_values.size() > 0);
  const int num_data = active_var_values.size();
  const int num_err_terms = num_data * polys.rows();

  std::set<VarType> all_vars;
  for (int i = 0; i < polys.rows(); i++) {
    const std::set<VarType> poly_vars = polys[i].getVariables();
    all_vars.insert(poly_vars.begin(), poly_vars.end());
  }

  // The set of vars left unspecified in at least one of active_var_values,
  // and thus which must appear in our return map.
  std::set<VarType> vars_to_estimate_set;
  for (const PartialEvalType& partial_eval_map : active_var_values) {
    std::set<VarType> unspecified_vars = all_vars;
    for (auto const& element : partial_eval_map) {
      unspecified_vars.erase(element.first);
    }
    vars_to_estimate_set.insert(unspecified_vars.begin(),
                                unspecified_vars.end());
  }
  std::vector<VarType> vars_to_estimate(vars_to_estimate_set.begin(),
                                        vars_to_estimate_set.end());
  int num_to_estimate = vars_to_estimate.size();

  // Make sure we have as many data points as vars we are estimating, or else
  // our solution will be meaningless.
  DRAKE_ASSERT(num_data >= num_to_estimate);

  // Build up our optimization problem's decision variables.
  OptimizationProblem problem;
  const auto parameter_variables =
      problem.AddContinuousVariables(num_to_estimate, "param");
  const auto error_variables =
      problem.AddContinuousVariables(num_err_terms, "error");

  // Create any necessary VarType IDs.  We build up two lists of VarType:
  //  * problem_vartypes holds a VarType for each decision variable.  This
  //    list will be used to build the constraints.
  //  * error_vartypes holds a VarType for each error variable.  This list
  //    will be used to build the objective function.
  // In addition a temporary set, vars_to_estimate_set, is used for ensuring
  // unique names.
  std::vector<VarType> problem_vartypes = vars_to_estimate;
  std::vector<VarType> error_vartypes;
  std::set<VarType> vars_in_problem = vars_to_estimate_set;
  for (int i = 0; i < num_err_terms; i++) {
    VarType error_var = CreateUnusedVar("err", vars_in_problem);
    vars_in_problem.insert(error_var);
    error_vartypes.push_back(error_var);
    problem_vartypes.push_back(error_var);
  }

  // For each datum, build a constraint with an error term.
  for (int datum_num = 0; datum_num < num_data; datum_num++) {
    VectorXExpr constraint_polys(polys.rows(), 1);
    const PartialEvalType& partial_eval_map = active_var_values[datum_num];
    for (int poly_num = 0; poly_num < polys.rows(); poly_num++) {
      ExprType partial_poly = polys[poly_num].evaluatePartial(partial_eval_map);
      ExprType constraint_poly =
          partial_poly + ExprType(1, error_vartypes[datum_num * polys.rows() +
                                                    poly_num]);
      constraint_polys[poly_num] = constraint_poly;
    }
    // The constraint behaviour depends on ExprType, so we dispatch on dynamic
    // type.  This will optimize out in each template instantiation.
    const VectorXPoly* constraints_as_polynomials =
        dynamic_cast<const VectorXPoly*>(&constraint_polys);
    if (constraints_as_polynomials) {
      problem.AddPolynomialConstraint(
          *constraints_as_polynomials, problem_vartypes,
          Eigen::VectorXd::Constant(polys.rows(), 0),
          Eigen::VectorXd::Constant(polys.rows(), 0));
    }
  }

  // Create a cost function that is least-squares on the error terms.
  auto cost = problem.AddQuadraticCost(
      Eigen::MatrixXd::Identity(num_err_terms, num_err_terms),
      Eigen::VectorXd::Zero(num_err_terms),
      std::list<DecisionVariableView> { error_variables });

  // Solve the problem and copy out the result.
  SolutionResult solution_result = problem.Solve();
  if (solution_result != kSolutionFound) {
    throw std::runtime_error(
        "Solution failed: " + std::to_string(solution_result));
  }
  PartialEvalType estimates;
  for (int i = 0; i < num_to_estimate; i++) {
    VarType var = vars_to_estimate[i];
    estimates[var] = parameter_variables.value()[i];
  }
  CoefficientType error_squared = 0;
  for (int i = 0; i < num_err_terms; i++) {
    error_squared += error_variables.value()[i] * error_variables.value()[i];
  }

  return std::make_pair(estimates, std::sqrt(error_squared));
}

}  // namespace solvers
}  // namespace drake

template class DRAKEOPTIMIZATION_EXPORT
drake::solvers::SystemIdentification<Polynomiald>;

template class DRAKEOPTIMIZATION_EXPORT
drake::solvers::SystemIdentification<TrigPolyd>;
