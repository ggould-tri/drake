#include "drake/common/dimensions.h"

#include <gtest/gtest.h>

#include "drake/common/eigen_types.h"
#include "drake/common/polynomial.h"

/// Test the most basic functionality of the Dimensions module to see if
/// things are working as expected.
GTEST_TEST(dimensions_test, SmokeTest) {
  typedef drake::Dimensioned<double> Quantity;
  const Quantity one_meter = Quantity::meters(1.);
  const Quantity one_second = Quantity::seconds(1.);
  const Quantity one_hertz = 1 / Quantity::seconds(1.);
  const Quantity one_kilogram = Quantity::kilograms(1.);
  const Quantity one_newton = Quantity::newtons(1.);

  EXPECT_EQ(one_meter * one_kilogram * one_hertz / one_second,
            one_newton);
  EXPECT_EQ((one_meter + one_meter), Quantity::meters(2.));
  EXPECT_DEATH(one_meter + one_second, "Tried to mix");
  EXPECT_DEATH(one_meter == one_second, "Tried to mix");
  EXPECT_EQ(one_meter, 1 * one_meter);
  EXPECT_EQ(one_meter, 1 * Quantity::meter());

  EXPECT_TRUE(one_meter < 2 * Quantity::meter());
  EXPECT_TRUE(one_meter <= 2 * Quantity::meter());
  EXPECT_TRUE(one_meter != 2 * Quantity::meter());
  EXPECT_TRUE(2 * Quantity::meter() > one_meter);
  EXPECT_TRUE(2 * Quantity::meter() >= one_meter);
}

/// Test that we can create an Eigen matrix of Dimensioned<double> and get
/// a sensible result.
GTEST_TEST(dimesions_test, MemberOfMatrix) {
  typedef drake::Dimensioned<double> Quantity;
  drake::Vector1<Quantity> meter_1x1;
  meter_1x1 << (1 * Quantity::meter());
  drake::Vector1<Quantity> square_meter_1x1 = meter_1x1 * meter_1x1;
  EXPECT_EQ(square_meter_1x1[0], 1 * Quantity::meter() * Quantity::meter());

  Eigen::Matrix<Quantity, 2, 1> meter_second_vector;
  meter_second_vector << 1 * Quantity::meter(), 1 * Quantity::second();
  auto meter_second_matrix =
      meter_second_vector * meter_second_vector.transpose();
  EXPECT_EQ(meter_second_matrix(0, 0),
            1 * Quantity::meter() * Quantity::meter());
  EXPECT_EQ(meter_second_matrix(0, 1),
            1 * Quantity::meter() * Quantity::second());
  EXPECT_EQ(meter_second_matrix(1, 0),
            1 * Quantity::meter() * Quantity::second());
  EXPECT_EQ(meter_second_matrix(1, 1),
            1 * Quantity::second() * Quantity::second());
}

/// Test that we can create a Dimensioned<Polynomial> and get a sensible result.
GTEST_TEST(dimensions_test, DimensionedPolynomial) {
  typedef Polynomial<double> PolyType;
  typedef drake::Dimensioned<PolyType> DimPoly;

  // Test basic operations
  DimPoly x = PolyType("x") * DimPoly::meter();
  DimPoly x_plus_one = x + (1 * DimPoly::meter());
  DimPoly y = PolyType("y") * DimPoly::second();
  EXPECT_EQ(x_plus_one * y, x * y + y * DimPoly::meter());

  EXPECT_DEATH(x / y, "abort");  // Polynomial doesn't provide division.
}
