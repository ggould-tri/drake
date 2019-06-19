#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "drake/common/default_scalars.h"
#include "drake/multibody/plant/element_collection.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/plant/test/multibody_plant_fixture.h"

namespace drake {
namespace multibody {
namespace {

// Test construction and destruction without elements.
// For simplicity this is the only testing we do with all of the scalartypes.
template <typename CollectionType>
void SmokeTestCollection() {
  CollectionType collection;
  EXPECT_EQ(collection.size(), 0);
  for (const auto x: collection) { EXPECT_TRUE(x != x); }
}
template<template<typename ScalarType> typename CollectionType>
void SmokeTestCollectionAllScalarTypes() {
  SmokeTestCollection<CollectionType<double>>();
  SmokeTestCollection<CollectionType<AutoDiffXd>>();
  SmokeTestCollection<CollectionType<symbolic::Expression>>();
}
GTEST_TEST(ElementCollection, SmokeTest) {
  SmokeTestCollectionAllScalarTypes<BodyCollection>();
  SmokeTestCollectionAllScalarTypes<JointCollection>();
  SmokeTestCollectionAllScalarTypes<JointActuatorCollection>();
  SmokeTestCollectionAllScalarTypes<FrameCollection>();
  SmokeTestCollection<ModelInstanceCollection>();
  SmokeTestCollection<ForceElementCollection>();
}

}  // namespace
}  // namespace multibody
}  // namespace drake
