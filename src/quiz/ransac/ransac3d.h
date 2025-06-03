#ifndef RANSAC3D_H
#define RANSAC3D_H

#include <unordered_set>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cstdlib>
#include <ctime>
#include <cmath>

template<typename PointT>
std::unordered_set<int> Ransac3d(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    for (int i = 0; i < maxIterations; i++)
    {
        std::unordered_set<int> inliers;

        // Randomly pick three unique points
        int ind1, ind2, ind3;
        do { ind1 = rand() % cloud->points.size(); } while(false);
        do { ind2 = rand() % cloud->points.size(); } while(ind2 == ind1);
        do { ind3 = rand() % cloud->points.size(); } while(ind3 == ind1 || ind3 == ind2);

        auto p1 = cloud->points[ind1];
        auto p2 = cloud->points[ind2];
        auto p3 = cloud->points[ind3];

        float A = (p2.y - p1.y)*(p3.z - p1.z) - (p2.z - p1.z)*(p3.y - p1.y);
        float B = (p2.z - p1.z)*(p3.x - p1.x) - (p2.x - p1.x)*(p3.z - p1.z);
        float C = (p2.x - p1.x)*(p3.y - p1.y) - (p2.y - p1.y)*(p3.x - p1.x);
        float D = -(A*p1.x + B*p1.y + C*p1.z);

        float denom = std::sqrt(A*A + B*B + C*C);
        if (denom == 0) continue;

        for (int idx = 0; idx < cloud->points.size(); idx++)
        {
            auto pt = cloud->points[idx];
            float dist = std::fabs(A*pt.x + B*pt.y + C*pt.z + D) / denom;

            if (dist < distanceTol)
                inliers.insert(idx);
        }

        if (inliers.size() > inliersResult.size())
            inliersResult = inliers;
    }

    return inliersResult;
}

#endif
