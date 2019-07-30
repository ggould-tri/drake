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
template <template<typename ScalarType> typename CollectionType, typename T>
void SmokeTestCollection() {
  MultibodyPlant<T> plant;
  CollectionType<T> collection(&plant);
  EXPECT_EQ(collection.size(), 0);
  for (const auto x: collection) { EXPECT_TRUE(x != x); }
}
template<template<typename ScalarType> typename CollectionType>
void SmokeTestCollectionAllScalarTypes() {
  SmokeTestCollection<CollectionType, double>();
  SmokeTestCollection<CollectionType, AutoDiffXd>();
  SmokeTestCollection<CollectionType, symbolic::Expression>();
}
GTEST_TEST(ElementCollection, SmokeTest) {
  SmokeTestCollectionAllScalarTypes<BodyCollection>();
  SmokeTestCollectionAllScalarTypes<JointCollection>();
  SmokeTestCollectionAllScalarTypes<JointActuatorCollection>();
  SmokeTestCollectionAllScalarTypes<FrameCollection>();
  SmokeTestCollectionAllScalarTypes<ModelInstanceCollection>();
  SmokeTestCollectionAllScalarTypes<ForceElementCollection>();
}

// Load up a fixture and take a look around an element collection.
TEST_F(MultibodyPlantFixture, SimpleBodyCollectionTest) {
  plant.Finalize();

  // Check that the begin/end iterators work right.
  constexpr int kNumBodies = 143;  // Number of bodies in the fixture.
  auto bodies = Bodies(&plant);
  EXPECT_EQ(bodies.size(), kNumBodies);
  EXPECT_EQ(bodies.size(), bodies.end() - bodies.begin());

  // Can we do a `for` loop?
  std::set<BodyIndex> seen;
  for (const Body<double>* body: bodies) {
    seen.insert(body->index());
  }
  EXPECT_EQ(seen.size(), kNumBodies);

  // Test out the "Named" selector (this also transitively tests "Filter").
  EXPECT_EQ(bodies.Named("pelvis").size(), 2);
  EXPECT_EQ(bodies.Named("main_body").OnlyOne().index(), mug->index());
}


// Do likewise for an index collection.
TEST_F(MultibodyPlantFixture, SimpleModelInstanceCollectionTest) {
  plant.Finalize();

  // Check that the begin/end iterators work right.
  constexpr int kNumModelInstances = 6;  // 2xatlas, mug, table, default, world
  auto model_instances = ModelInstances(&plant);
  EXPECT_EQ(model_instances.size(), kNumModelInstances);
  EXPECT_EQ(model_instances.size(),
            model_instances.end() - model_instances.begin());

  // Can we do a `for` loop?
  std::set<ModelInstanceIndex> seen;
  for (const ModelInstanceIndex model_instance: model_instances) {
    seen.insert(model_instance);
  }
  EXPECT_EQ(seen.size(), kNumModelInstances);

  // Test out the "Named" selector (this also transitively tests "Filter").
  EXPECT_EQ(model_instances.Named("Atlas1").size(), 1);
  EXPECT_EQ(model_instances.Named("Atlas2").OnlyOne(), atlas_model2);

  // Test that we can construct a ForceElementCollection even though it is not
  // fully specialized (has no `Named`).
  ForceElementCollection<double> force_elements = ForceElements(&plant);
  EXPECT_EQ(force_elements.size(), 1);  // Gravity exists.
  // If we said `force_elements.Named("foo");` here, we'd get a compile error,
  // which is correct.  I can't unit test that something doesn't compile, so
  // you have to take my word for it.
}

}  // namespace
}  // namespace multibody
}  // namespace drake
