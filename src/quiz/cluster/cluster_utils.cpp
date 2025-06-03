#include "cluster_utils.h"
#include "kdtree.h"           // needs search()
#include <algorithm>          // std::find

// ───────────────────────── helpers ──────────────────────────
static void proximity(const std::vector<std::vector<float>>& points,
                      int idx,
                      std::vector<int>& cluster,
                      std::vector<int>& processed,
                      KdTree* tree,
                      float distanceTol)
{
    processed.push_back(idx);
    cluster.push_back(idx);

    std::vector<int> nearby = tree->search(points[idx], distanceTol);
    for (int id : nearby)
        if (std::find(processed.begin(), processed.end(), id) == processed.end())
            proximity(points, id, cluster, processed, tree, distanceTol);
}

// ─────────────────── exported clustering api ─────────────────
std::vector<std::vector<int>> euclideanCluster(
        const std::vector<std::vector<float>>& points,
        KdTree* tree,
        float distanceTol)
{
    std::vector<std::vector<int>> clusters;
    std::vector<int> processed;

    for (int i = 0; i < points.size(); ++i)
        if (std::find(processed.begin(), processed.end(), i) == processed.end())
        {
            std::vector<int> cluster;
            proximity(points, i, cluster, processed, tree, distanceTol);
            clusters.push_back(cluster);
        }

    return clusters;
}
