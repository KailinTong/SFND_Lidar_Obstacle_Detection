//
// Created by tongadm on 01.08.21.
//
#include "cluster.h"

void proximity(const std::vector<std::vector<float>> &points, int point_id, std::vector<int> &cluster,
               std::vector<bool> &processed, KdTree *tree, float distanceTol) {
    processed.at(point_id) = true;
    auto point = points.at(point_id);
    cluster.emplace_back(point_id);
    auto ids = tree->search(point, distanceTol, 2);

    for(auto &id: ids){
        if(!processed.at(id)){
            proximity(points, id, cluster, processed, tree, distanceTol);
        }
    }


}

std::vector<std::vector<int>>
euclideanCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int minSize, int maxSize) {

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> processed(points.size(), false);


    for (int i = 0; i < points.size(); i++){
        if(!processed.at(i)){
            std::vector<int> cluster;   // create a new cluster
            proximity(points, i, cluster, processed, tree, distanceTol);
            clusters.emplace_back(cluster);
            if(cluster.size() < minSize) // TODO maybe this will neglect some useful clusters??
                continue;
            if(cluster.size() > maxSize)
                continue;
            clusters.emplace_back(cluster);
        }
    }

    return clusters;

}
