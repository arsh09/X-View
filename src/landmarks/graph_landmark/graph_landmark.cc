#include <x_view_core/landmarks/graph_landmark/graph_landmark.h>

#include <x_view_core/landmarks/graph_landmark/blob_extractor.h>
#include <x_view_core/landmarks/graph_landmark/graph_builder.h>
#include <x_view_core/x_view_locator.h>

namespace x_view {

GraphLandmark::GraphLandmark(const FrameData& frame_data)
    : AbstractSemanticLandmark(frame_data) {

  const auto& parameters = Locator::getParameters();
  const auto& landmark_parameters =
      parameters->getChildPropertyList("landmark");
  const auto& timer = Locator::getTimer();

  // Graph descriptor filled up by this function
  Graph descriptor;

  // *********** Blobs extraction ********** //

  BlobExtractorParams blob_extractor_params;
  blob_extractor_params.dilate_and_erode =
      landmark_parameters->getBoolean("dilate_and_erode", true);
  blob_extractor_params.num_dilate_reps =
      landmark_parameters->getInteger("num_dilate", 4);
  blob_extractor_params.num_erode_reps =
      landmark_parameters->getInteger("num_erode", 4);

  const std::string blob_filter_type =
      landmark_parameters->getString("blob_filter_type", "ABSOLUTE");

  if(blob_filter_type == "ABSOLUTE") {
    blob_extractor_params.blob_size_filtering.type =
        BlobExtractorParams::MIN_BLOB_SIZE_TYPE::ABSOLUTE;
    const int min_blob_size =
        landmark_parameters->getInteger("min_blob_size", 300);
    blob_extractor_params.blob_size_filtering.num_min_pixels = min_blob_size;
  } else if(blob_filter_type == "RELATIVE") {
    blob_extractor_params.blob_size_filtering.type =
    BlobExtractorParams::MIN_BLOB_SIZE_TYPE::RELATIVE;
    const real_t fraction_min_blob =
        landmark_parameters->getFloat("min_blob_size", 0.05);
    blob_extractor_params.blob_size_filtering.fraction_min_pixels =
        fraction_min_blob;
  } else {
    LOG(ERROR) << "Graph landmark could not recognize <" << blob_filter_type
               << "> as 'blob_filter_type' parameter.";
  }

  timer->registerTimer("BlobExtraction", "SemanticLandmarkExtraction");
  timer->start("BlobExtraction");
  image_blobs_ = BlobExtractor::extractBlobs(semantic_image_,
                                             blob_extractor_params);
  timer->stop("BlobExtraction");


  // *********** Graph generation ********** //

  GraphBuilderParams graph_builder_params;
  const std::string graph_extraction_type =
      landmark_parameters->getString("extraction_type", "IMAGE");

  if(graph_extraction_type == "IMAGE") {
    graph_builder_params.extraction_type =
        GraphBuilderParams::EXTRACTION_TYPE::EDGES_DEFINED_ON_BLOB_NEIGHBORS;

    graph_builder_params.max_distance_for_neighborhood =
        landmark_parameters->getInteger("blob_neighbor_distance", 10);

  } else if(graph_extraction_type == "3D_SPACE") {
    graph_builder_params.extraction_type =
        GraphBuilderParams::EXTRACTION_TYPE::EDGES_DEFINED_ON_3D_SPACE;

    graph_builder_params.max_euclidean_distance =
        landmark_parameters->getFloat("max_euclidean_distance", 2.f);
  } else {
    LOG(ERROR) << "Unrecognized graph extraction type <"
               << graph_extraction_type << ">.";
  }

  timer->registerTimer("GraphBuilding", "SemanticLandmarkExtraction");
  timer->start("GraphBuilding");
  descriptor = GraphBuilder::extractSemanticGraph(frame_data,
                                                  image_blobs_,
                                                  graph_builder_params);
  timer->stop("GraphBuilding");

  // Create the descriptor stored in this landmark by generating a
  // VectorDescriptor containing the graph data.
  descriptor_ = std::make_shared<GraphDescriptor>(GraphDescriptor(descriptor));

}

}

