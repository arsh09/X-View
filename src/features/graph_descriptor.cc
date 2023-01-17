#include <x_view_core/features/graph_descriptor.h>

namespace x_view {

GraphDescriptor::GraphDescriptor(const Graph& descriptor)
    : AbstractDescriptor(),
      descriptor_(descriptor) {}
}
