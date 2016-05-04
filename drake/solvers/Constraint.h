#pragma once

#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/SparseCore>

#include "drake/util/Polynomial.h"

namespace Drake {

/* Constraint
 * @brief A constraint is a function + lower and upper bounds.
 *
 * Some thoughts:
 * It should support evaluating the constraint, adding it to an optimization
 *problem,
 * and have support for constraints that require slack variables (adding
 *additional decision variables to the problem).  There
 * should also be some notion of parameterized constraints:  e.g. the
 *acceleration constraints in the rigid body dynamics are constraints
 * on vdot and f, but are "parameterized" by q and v.
 */
class Constraint {
  void check(size_t num_constraints) {
    static_cast<void>(num_constraints);
    assert(lower_bound_.size() == num_constraints &&
           "Size of lower bound must match number of constraints.");
    assert(upper_bound_.size() == num_constraints &&
           "Size of upper bound must match number of constraints.");
  }

 public:
  explicit Constraint(size_t num_constraints)
      : lower_bound_(num_constraints), upper_bound_(num_constraints) {
    check(num_constraints);
    lower_bound_.setConstant(-std::numeric_limits<double>::infinity());
    upper_bound_.setConstant(std::numeric_limits<double>::infinity());
  }

  template <typename DerivedLB, typename DerivedUB>
  Constraint(size_t num_constraints, Eigen::MatrixBase<DerivedLB> const& lb,
             Eigen::MatrixBase<DerivedUB> const& ub)
      : lower_bound_(lb), upper_bound_(ub) {
    check(num_constraints);
  }

  virtual ~Constraint() {}

  // TODO(bradking): consider using a Ref for `y`.  This will require the client
  // to do allocation, but also allows it to choose stack allocation instead.
  virtual void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const = 0;
  virtual void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const = 0;  // move this to
                                                // DifferentiableConstraint
                                                // derived class if/when we need
                                                // to support non-differentiable
                                                // functions

  Eigen::VectorXd const& lower_bound() const { return lower_bound_; }
  Eigen::VectorXd const& upper_bound() const { return upper_bound_; }
  size_t num_constraints() const { return lower_bound_.size(); }

 protected:
  Eigen::VectorXd lower_bound_, upper_bound_;
};

/** QuadraticConstraint
 * @brief  lb <= .5 x'Qx + b'x <= ub
 */
class QuadraticConstraint : public Constraint {
 public:
  static const int kNumConstraints = 1;

  template <typename DerivedQ, typename Derivedb>
  QuadraticConstraint(const Eigen::MatrixBase<DerivedQ>& Q,
                      const Eigen::MatrixBase<Derivedb>& b, double lb,
                      double ub)
      : Constraint(kNumConstraints,
                   Vector1d::Constant(lb), Vector1d::Constant(ub)),
        Q_(Q),
        b_(b) {}

  ~QuadraticConstraint() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = .5 * x.transpose() * Q_ * x + b_.transpose() * x;
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = .5 * x.transpose() * Q_.cast<TaylorVarXd>() * x +
        b_.cast<TaylorVarXd>().transpose() * x;
  };

 private:
  Eigen::MatrixXd Q_;
  Eigen::VectorXd b_;
};

/** PolynomialConstraint
 * @brief lb <= P(x, y...) <= ub where P is a multivariate polynomial
 *  in x, y...
 *
 * A constraint on the value of a multivariate polynomial.
 *
 * The Polynomial class uses a different variable naming scheme; thus the
 * caller must provide a list of Polynomial::VarType variables that correspond
 * to the members of the OptimizationProblem::Binding (the individual scalar
 * elements of the given VariableList).
 */
class PolynomialConstraint : public Constraint {
 public:
  static const int kNumConstraints = 1;

  PolynomialConstraint(
      const Polynomiald& polynomial,
      const std::vector<Polynomiald::VarType>& poly_vars,
      double lb, double ub)
      : Constraint(kNumConstraints,
                   Vector1d::Constant(lb), Vector1d::Constant(ub)),
        polynomial_(polynomial),
        poly_vars_(poly_vars) {}

  ~PolynomialConstraint() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    double_evaluation_point_.clear();
    for (int i = 0; i < poly_vars_.size(); i++) {
      double_evaluation_point_[poly_vars_[i]] = x[i];
    }
    y.resize(num_constraints());
    y[0] = polynomial_.evaluateMultivariate(double_evaluation_point_);
  }

  void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    taylor_evaluation_point_.clear();
    for (int i = 0; i < poly_vars_.size(); i++) {
      taylor_evaluation_point_[poly_vars_[i]] = x[i];
    }
    y.resize(num_constraints());
    y[0] = polynomial_.evaluateMultivariate(taylor_evaluation_point_);
  }

 private:
  const Polynomiald polynomial_;
  const std::vector<Polynomiald::VarType> poly_vars_;

  /// To avoid repeated allocation, reuse a map for the evaluation point.
  mutable std::map<Polynomiald::VarType, double> double_evaluation_point_;
  mutable std::map<Polynomiald::VarType, TaylorVarXd> taylor_evaluation_point_;
};

...

// todo: consider implementing DifferentiableConstraint,
// TwiceDifferentiableConstraint, ComplementarityConstraint,
// IntegerConstraint, ...
/** LinearConstraint
 * @brief Implements a constraint of the form @f lb <= Ax <= ub @f
 */
class LinearConstraint : public Constraint {
 public:
  explicit LinearConstraint(size_t num_constraints)
      : Constraint(num_constraints) {}
  template <typename DerivedA, typename DerivedLB, typename DerivedUB>
  LinearConstraint(const Eigen::MatrixBase<DerivedA>& a,
                   const Eigen::MatrixBase<DerivedLB>& lb,
                   const Eigen::MatrixBase<DerivedUB>& ub)
      : Constraint(a.rows(), lb, ub), A_(a) {
    assert(a.rows() == lb.rows());
  }

  ~LinearConstraint() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = A_ * x;
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = A_.cast<TaylorVarXd>() * x;
  };

  virtual Eigen::SparseMatrix<double> GetSparseMatrix() const {
    return A_.sparseView();
  }
  virtual const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>&
  A() const {
    return A_;
  }

 protected:
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A_;
};

/** LinearEqualityConstraint
 * @brief Implements a constraint of the form @f Ax = b @f
 */
class LinearEqualityConstraint : public LinearConstraint {
 public:
  template <typename DerivedA, typename DerivedB>
  LinearEqualityConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                           const Eigen::MatrixBase<DerivedB>& beq)
      : LinearConstraint(Aeq, beq, beq) {}

  ~LinearEqualityConstraint() override {}

  /* updateConstraint
   * @brief change the parameters of the constraint (A and b), but not the
   *variable associations
   *
   * note that A and b can change size in the rows only (representing a
   *different number of linear constraints, but on the same decision variables)
   */
  template <typename DerivedA, typename DerivedB>
  void updateConstraint(const Eigen::MatrixBase<DerivedA>& Aeq,
                        const Eigen::MatrixBase<DerivedB>& beq) {
    assert(Aeq.rows() == beq.rows());
    if (Aeq.cols() != A_.cols())
      throw std::runtime_error("Can't change the number of decision variables");
    A_.resize(Aeq.rows(), Eigen::NoChange);
    A_ = Aeq;
    lower_bound_.conservativeResize(beq.rows());
    lower_bound_ = beq;
    upper_bound_.conservativeResize(beq.rows());
    upper_bound_ = beq;
  }
};

/** BoundingBoxConstraint
*@brief Implements a constraint of the form @f lb <= x <= ub @f
*Note: the base Constraint class (as implemented at the moment) could play this
* role.  But this class enforces
*that it is ONLY a bounding box constraint, and not something more general.
*/
class BoundingBoxConstraint : public LinearConstraint {
 public:
  template <typename DerivedLB, typename DerivedUB>
  BoundingBoxConstraint(const Eigen::MatrixBase<DerivedLB>& lb,
                        const Eigen::MatrixBase<DerivedUB>& ub)
      : LinearConstraint(Eigen::MatrixXd::Identity(lb.rows(), lb.rows()), lb,
                         ub) {}

  ~BoundingBoxConstraint() override {}

  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = x;
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = x;
  }
};


/** LinearComplementarityConstraint
 *
 * Implements a constraint of the form:
 * <pre>
 *   Mx + q >= 0
 *   x >= 0
 *   x'(Mx + q) == 0
 * </pre>
 * An implied slack variable complements any 0 component of x.  To get
 * the slack values at a given solution x, use eval(x).
 */
class LinearComplementarityConstraint : public Constraint {
 public:
  template <typename DerivedM, typename Derivedq>
  LinearComplementarityConstraint(const Eigen::MatrixBase<DerivedM>& M,
                                  const Eigen::MatrixBase<Derivedq>& q)
      : Constraint(q.rows()), M_(M), q_(q) {}

  ~LinearComplementarityConstraint() override {}

  /** Return Mx + q (the value of the slack variable). */
  void eval(const Eigen::Ref<const Eigen::VectorXd>& x,
                    Eigen::VectorXd& y) const override {
    y.resize(num_constraints());
    y = (M_ * x) + q_;
  }
  void eval(const Eigen::Ref<const TaylorVecXd>& x,
                    TaylorVecXd& y) const override {
    y.resize(num_constraints());
    y = (M_.cast<TaylorVarXd>() * x) + q_.cast<TaylorVarXd>();
  };

  const Eigen::MatrixXd& M() const { return M_; }
  const Eigen::VectorXd& q() const { return q_; }

 private:
  // TODO(ggould-tri) We are storing what are likely statically sized matrices
  // in dynamically allocated containers.  This probably isn't optimal.
  Eigen::MatrixXd M_;
  Eigen::VectorXd q_;
};

}  // namespace Drake
