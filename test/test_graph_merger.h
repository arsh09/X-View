#ifndef X_VIEW_TEST_GRAPH_MERGER_H
#define X_VIEW_TEST_GRAPH_MERGER_H

#include <x_view_core/features/graph.h>

using namespace x_view;

namespace x_view_test {

/**
 * \brief Merges the two graphs passed as argument and generated a new graph
 * containing the merged version of the two.
 * \param g1 First graph to be merged.
 * \param g2 Second graph to be merged.
 * \param merged Pointer to the generated graph representing the merged
 * version of the two graphs passed as argument.
 */
void mergeGraphs(const Graph& g1, const Graph g2, Graph* merged);

}

#endif //X_VIEW_TEST_GRAPH_MERGER_H
