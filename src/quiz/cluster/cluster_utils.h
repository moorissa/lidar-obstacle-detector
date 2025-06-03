#ifndef CLUSTER_UTILS_H
#define CLUSTER_UTILS_H

#include <vector>

// Forward declaration so we don’t pull kdtree internals here.
struct KdTree;

/**
 * Euclidean clustering.
 * @param points       xyz coordinates, one point per entry.
 * @param tree         pointer to an already-filled KdTree built on `points`.
 * @param distanceTol  clustering radius (same units as your coordinates).
 * @return vector of clusters; each cluster is a vector of point indices.
 */
std::vector<std::vector<int>> euclideanCluster(
        const std::vector<std::vector<float>>& points,
        KdTree* tree,
        float distanceTol);

#endif  // CLUSTER_UTILS_H
