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
  /// Construct a zero-element collection.
  MultibodyElementCollection() {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyElementCollection)

  using ThisType = MultibodyElementCollection<ElementType<T>, ElementIndexType>;
  using ElementFilterFunc = std::function<bool(const ElementType<T>&)>;
  using IndexFilterFunc = std::function<bool(ElementIndexType)>;
  using ConstIteratorType =
      typename std::vector<const ElementType<T>*>::const_iterator;

  inline ConstIteratorType begin() const { return elements_.begin(); }
  inline ConstIteratorType end() const { return elements_.end(); }
  int size() const { return static_cast<int>(elements_.size()); }

  ThisType Filter(ElementFilterFunc func) const {
    auto new_elements = elements_;
    std::remove_if(new_elements.begin(), new_elements.end(),
                   [&](const auto& element){ return !func(*element); });
    return ThisType(new_elements);
  }

 private:
  explicit MultibodyElementCollection(
      const std::vector<const ElementType<T>*>& elements)
      : elements_(elements) {}

  std::vector<const ElementType<T>*> elements_;
};

template <typename T>
using BodyCollection = MultibodyElementCollection<Body<T>, BodyIndex>;
template class MultibodyElementCollection<Body<double>, BodyIndex>;

template <typename T>
using JointCollection = MultibodyElementCollection<Joint<T>, JointIndex>;
template class MultibodyElementCollection<Joint<double>, JointIndex>;

template <typename T>
using JointActuatorCollection =
    MultibodyElementCollection<JointActuator<T>, JointIndex>;
template class MultibodyElementCollection<JointActuator<double>, JointIndex>;

template <typename T>
using FrameCollection = MultibodyElementCollection<Frame<T>, FrameIndex>;
template class MultibodyElementCollection<Frame<double>, FrameIndex>;


/// A generic class for vector-like collections of MultibodyTree TypeSafeIndex
/// types with no public element type (eg, `ModelIndexInstance`).
template <typename ElementIndexType>
class MultibodyIndexCollection {
 public:
  /// Construct a zero-element collection.
  MultibodyIndexCollection() {}

  DRAKE_DEFAULT_COPY_AND_MOVE_AND_ASSIGN(MultibodyIndexCollection)

  using ThisType = MultibodyIndexCollection<ElementIndexType>;
  using IndexFilterFunc = std::function<bool(ElementIndexType)>;
  using ConstIteratorType =
      typename std::vector<ElementIndexType>::const_iterator;

  inline ConstIteratorType begin() const { return indexes_.begin(); }
  inline ConstIteratorType end() const { return indexes_.end(); }
  int size() const { return static_cast<int>(indexes_.size()); }

  ThisType Filter(IndexFilterFunc func) const {
    auto new_indexes = indexes_;
    std::remove_if(new_indexes.begin(), new_indexes.end(),
                   [&](const auto& index){ return !func(*index); });
    return ThisType(new_indexes);
  }

 private:
  explicit MultibodyIndexCollection(
      const std::vector<ElementIndexType>& indexes)
      : indexes_(indexes) {}

  std::vector<ElementIndexType> indexes_;
};

using ModelInstanceCollection = MultibodyIndexCollection<ModelInstanceIndex>;
using ForceElementCollection = MultibodyIndexCollection<ForceElementIndex>;

}  // namespace multibody
}  // namespace drake
