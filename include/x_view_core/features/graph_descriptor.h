#ifndef X_VIEW_GRAPH_DESCRIPTOR_H
#define X_VIEW_GRAPH_DESCRIPTOR_H

#include <x_view_core/features/abstract_descriptor.h>
#include <x_view_core/features/graph.h>

namespace x_view {

/**
 * \brief This class encapsulates all types of descriptors that might be
 * represented as graph.
 */
class GraphDescriptor : public AbstractDescriptor {

 public:

  GraphDescriptor(const Graph& descriptor);
  virtual ~GraphDescriptor() {}

  const Graph& getDescriptor() const {
    return descriptor_;
  }

 protected:
  const Graph descriptor_;
};

}

#endif //X_VIEW_GRAPH_DESCRIPTOR_H
