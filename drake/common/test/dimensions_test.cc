#include "drake/common/dimensions.h"

#include "gtest/gtest.h"


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
  EXPECT_DEATH(one_meter + one_second, "assertion");
  EXPECT_DEATH(one_meter == one_second, "assertion");
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
  // TODO(ggould-tri) Write this test.
}

/// Test that we can create a Dimensioned<Polynomial> and get a sensible result.
GTEST_TEST(dimensions_test, DimensionedPolynomial) {
  // TODO(ggould-tri) Write this test.
}
