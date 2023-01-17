#ifndef X_VIEW_ABSTRACT_DESCRIPTOR_H
#define X_VIEW_ABSTRACT_DESCRIPTOR_H

#include <x_view_core/x_view_types.h>

namespace x_view {

/**
 * \brief Internal representation of a descriptor.
 * Each descriptor used in XView must implement this interface.
 * \details The introduction of AbstractDescriptor allow XView to operate
 * seamlessly with different features such as 'vector-based' descriptors
 * (histograms, visual descriptors, etc) an with more complex ones such as
 * 'graph-based' descriptors.
 */
class AbstractDescriptor {

 public:
  AbstractDescriptor();

  // trick to make AbstractDescriptor a pure virtual class with no need to
  // introduce useless abstract methods which must be implemented in the
  // subclass.
  virtual ~AbstractDescriptor() = 0;

}; // AbstractDescriptor


}
#endif //X_VIEW_ABSTRACT_DESCRIPTOR_H
