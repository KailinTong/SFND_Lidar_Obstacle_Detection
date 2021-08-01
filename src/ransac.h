/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include <unordered_set>
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker

#ifndef PLAYBACK_Ransac_H
#define PLAYBACK_Ransac_H

template<typename PointT>
pcl::PointIndices::Ptr Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    int size_p = cloud->points.size();
    // For max iterations
    int max_num_inliers = 0;

    for(auto iter = 0; iter < maxIterations; iter++) {
        std::unordered_set<int> inliersTemp;

        while(inliersTemp.size() < 3) 	//points must be more than 3 otherwise there are segmentation error
            inliersTemp.insert(rand() % (size_p - 0) + 0);

        auto itr = inliersTemp.begin();

        auto p_1 = cloud->points.at(*itr);
        itr++;
        auto p_2 = cloud->points.at(*itr);
        itr++;
        auto p_3 = cloud->points.at(*itr);

        float x1 = p_1.x;
        float y1 = p_1.y;
        float z1 = p_1.z;
        float x2 = p_2.x;
        float y2 = p_2.y;
        float z2 = p_2.z;
        float x3 = p_3.x;
        float y3 = p_3.y;
        float z3 = p_3.z;



        float A = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        float B = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        float C = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
//          v1 = < x2 - x1, y2 - y1, z2 - z1 >
//          v2 = < x3 - x1, y3 - y1, z3 - z1 >
//        v1 \times v2 = <(y2−y1)(z3−z1)−(z2−z1)(y3−y1), (z2−z1)(x3−x1)−(x2−x1)(z3−z1), (x2−x1)(y3−y1)−(y2−y1)(x3−x1)>

        float D = - (A * x1 + B * y1 + C * z1);
//        A = i,
//        B = j,
//        C = k,
//        D = -( ix1 + jy1 + kz1 )


        // Measure distance between every point and fitted line  d = |Ax+By+C|/sqrt(A^2+B^2)
        for (int index = 0; index < cloud->points.size(); index++) {

            if(inliersTemp.count(index) > 0){  // the index is already in inliersTemp
                continue;
            }

            auto point = cloud->points.at(index);
            double d = std::abs(A * point.x + B * point.y + C * point.z + D) / std::sqrt(A * A + B * B + C * C);

//            d = |A*x+B*y+C*z+D|/sqrt(A^2+B^2+C^2)

            if (d < distanceTol) {
                inliersTemp.emplace(index);
            }
        }
        // If distance is smaller than threshold count it as inlier

        if (inliersTemp.size() > inliersResult.size()) {
            inliersResult.swap(inliersTemp);
        }
    }


    pcl::PointIndices::Ptr finalInliers (new pcl::PointIndices ());
    for(const auto &index: inliersResult){
        finalInliers->indices.emplace_back(index);
    }


    // Return indicies of inliers from fitted line with most inliers

    return finalInliers;

}

#endif //PLAYBACK_Ransac_H
