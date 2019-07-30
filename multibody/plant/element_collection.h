#pragma once

#include <algorithm>
#include <functional>
#include <vector>

#include "drake/common/drake_assert.h"
#include "drake/common/drake_copyable.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/body.h"
#include "drake/multibody/tree/multibody_tree.h"
#include "drake/multibody/tree/multibody_tree_element.h"
#include "drake/multibody/tree/multibody_tree_indexes.h"

namespace drake {
namespace multibody {

// Make our template be a partial specialization of itself to elide scalartype
// on the derived class; see `multibody_tree_element.h` for a more detailed
// description of this trick.
/// @cond
template <class ElementType, typename ElementIndexType>
class MultibodyElementCollection;
/// @endcond

/// A generic class for vector-like collections of MultibodyTreeElement that
/// have a public element type (eg, `Body`).
template <template <typename> class ElementType,
    typename T, typename ElementIndexType>
class MultibodyElementCollection<ElementType<T>, ElementIndexType> {
 public:
  using ThisType = MultibodyElementCollection<ElementType<T>, ElementIndexType>;
  using ElementFilterFunc = std::function<bool(const ElementType<T>&)>;
  using IndexFilterFunc = std::function<bool(ElementIndexType)>;
  using ElementLookupFunc =
      std::function<const ElementType<T>&(ElementIndexType)>;
  using ElementVecType = std::vector<const ElementType<T>*>;
  using ConstIteratorType = typename ElementVecType::const_iterator;

  /// Construct a collection of no elements
  ///
  /// @p plant will be aliased and must remain valid for the lifetime of this
  /// collection.
  explicit MultibodyElementCollection(const MultibodyPlant<T>* plant)
      : plant_(plant) {}

  /// Given an MBP, a lookup function (eg, `MultibodyPlant::get_body`), and a
  /// maximum index (eg, `MultibodyPlant::num_bodies`), return an
  /// `ElementCollection` with all of the elements.
  static ThisType CollectAll(const MultibodyPlant<T>* plant,
                             ElementLookupFunc id_lookup_func,
                             int num_indexes) {
    ElementVecType elements;
    for (int i = 0; i < num_indexes; i++) {
      elements.push_back(&id_lookup_func(ElementIndexType(i)));
    }
    return ThisType(plant, elements);
  }

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyElementCollection)

  inline ConstIteratorType begin() const { return elements_.begin(); }
  inline ConstIteratorType end() const { return elements_.end(); }
  int size() const { return static_cast<int>(elements_.size()); }

  /// Assert that there is only one member of this collection and return it.
  const ElementType<T>& OnlyOne() const {
    DRAKE_DEMAND(size() == 1);
    return **begin();
  }

  /// @return a collection of all members `m` of this collection for which
  /// `func(m)` is true.
  // TODO(ggould) Just use `remove_copy_if` once we have C++20.
  ThisType Filter(ElementFilterFunc func) const {
    ElementVecType new_elements = elements_;
    new_elements.erase(
        std::remove_if(new_elements.begin(), new_elements.end(),
                       [&](const auto& element){ return !func(*element); }),
        new_elements.end());
    return ThisType(plant_, new_elements);
  }

  /// @return a collection of all members `m` of this collection for which
  /// `m.name()` is @p name.
  ThisType Named(const std::string& name) const {
    return Filter([name](const ElementType<T>& e) { return e.name() == name; });
  }

 private:
  MultibodyElementCollection(
      const MultibodyPlant<T>* plant,
      const std::vector<const ElementType<T>*>& elements)
      : plant_(plant),
        elements_(elements) {}

  const MultibodyPlant<T>* plant_;
  const ElementVecType elements_;
};


template <typename T>
using JointCollection = MultibodyElementCollection<Joint<T>, JointIndex>;

template <typename T>
using BodyCollection = MultibodyElementCollection<Body<T>, BodyIndex>;

template <typename T>
BodyCollection<T> Bodies(const MultibodyPlant<T>* plant) {
  return BodyCollection<T>::CollectAll(
      plant,
      [&](BodyIndex id) -> const Body<T>& { return plant->get_body(id); },
      plant->num_bodies());
}

template <typename T>
using JointActuatorCollection =
    MultibodyElementCollection<JointActuator<T>, JointIndex>;

template <typename T>
using FrameCollection = MultibodyElementCollection<Frame<T>, FrameIndex>;


/// A generic class for vector-like collections of MultibodyTree TypeSafeIndex
/// types with no public element type (eg, `ModelIndexInstance`).
template <typename ElementIndexType, typename T>
class MultibodyIndexCollection {
 public:
  using ThisType = MultibodyIndexCollection<ElementIndexType, T>;
  using IndexFilterFunc = std::function<bool(ElementIndexType)>;
  using ConstIteratorType =
      typename std::vector<ElementIndexType>::const_iterator;

  /// Construct a collection of no elements
  ///
  /// @p plant will be aliased and must remain valid for the lifetime of this
  /// collection.
  explicit MultibodyIndexCollection(const MultibodyPlant<T>* plant)
      : plant_(plant) {}

  MultibodyIndexCollection(
      const MultibodyPlant<T>* plant,
      const std::vector<ElementIndexType>& indexes)
      : plant_(plant),
        indexes_(indexes) {}

  /// Given an MBP and a maximum index (eg, `MultibodyPlant::num_bodies`),
  /// return an `ElementCollection` with all of the elements.
  static ThisType CollectAll(const MultibodyPlant<T>* plant,
                             int num_indexes) {
    std::vector<ElementIndexType> indexes;
    for (int i = 0; i < num_indexes; i++) {
      indexes.push_back(ElementIndexType(i));
    }
    return ThisType(plant, indexes);
  }

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyIndexCollection)

  inline ConstIteratorType begin() const { return indexes_.begin(); }
  inline ConstIteratorType end() const { return indexes_.end(); }
  int size() const { return static_cast<int>(indexes_.size()); }

  /// Assert that there is only one member of this collection and return it.
  ElementIndexType OnlyOne() const {
    DRAKE_DEMAND(size() == 1);
    return *begin();
  }

  /// @return a collection of all members `id` of this collection for which
  /// `func(id)` is true.
  ThisType Filter(IndexFilterFunc func) const {
    std::vector<ElementIndexType> new_indexes = indexes_;
    new_indexes.erase(
        std::remove_if(new_indexes.begin(), new_indexes.end(),
                       [&](const auto& index){ return !func(index); }),
        new_indexes.end());
    return ThisType(plant_, new_indexes);
  }

  /// Implicit partial specialization: ModelInstanceIndex'es have names.
  ThisType Named(const std::string& name) const {
    return Filter([&](ElementIndexType id) {
        return plant_->GetModelInstanceName(id) == name; });
  }

 private:
  const MultibodyPlant<T>* plant_;
  std::vector<ElementIndexType> indexes_;
};

template <typename T>
using ModelInstanceCollection = MultibodyIndexCollection<ModelInstanceIndex, T>;

template <typename T>
ModelInstanceCollection<T> ModelInstances(const MultibodyPlant<T>* plant) {
  return ModelInstanceCollection<T>::CollectAll(
      plant,
      plant->num_model_instances());
}

template <typename T>
using ForceElementCollection = MultibodyIndexCollection<ForceElementIndex, T>;

template <typename T>
ForceElementCollection<T> ForceElements(const MultibodyPlant<T>* plant) {
  return ForceElementCollection<T>::CollectAll(
      plant,
      plant->num_force_elements());
}

}  // namespace multibody
}  // namespace drake
