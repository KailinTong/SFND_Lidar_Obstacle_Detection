//
// Created by tongadm on 01.08.21.
//
#include "mykdtree.h"

#ifndef PLAYBACK_CLUSTER_H
#define PLAYBACK_CLUSTER_H

void proximity(const std::vector<std::vector<float>> &points, int point_id, std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol);;
std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);
#endif //PLAYBACK_CLUSTER_H
