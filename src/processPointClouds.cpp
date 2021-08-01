// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "cluster.h"
#include "ransac.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>()); // TODO why here is typename given?
    // Create the filtering object
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes, filterRes, filterRes);
    vg.filter (*cloud_filtered);


    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> region(true); // TODO why here is typename not given?
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_region);


    std::vector<int> indices;

    //To remove the points above the lidar head (roof points).
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -0.1, 1));
    roof.setInputCloud(cloud_region);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract; //created an object extract.
    extract.setInputCloud (cloud_region); //passing the filtered Cloud Regions
    extract.setIndices (inliers); //passing inliers created previously
    extract.setNegative (true);
    extract.filter (*cloud_region); //to filter out the roof points


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud (new pcl::PointCloud<PointT>());  // TODO new  pcl::PointCloud<PointT>() v.s. new pcl::PointCloud<PointT>
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());


    pcl::ExtractIndices<PointT> extract; //created an object extract.
    extract.setInputCloud (cloud); //passing cloud inputs
    extract.setIndices (inliers); //passing inliers created previously
    extract.setNegative (false);
    extract.filter (*planeCloud); //to filter out the plane
    extract.setNegative (true);
    extract.filter (*obstacleCloud); //to filter out the obstacles

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.
//    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
//    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
//    // Create the segmentation object
//    pcl::SACSegmentation<PointT> seg;
//    // Optional
//    seg.setOptimizeCoefficients (true);
//    // Mandatory
//    seg.setModelType (pcl::SACMODEL_PLANE);
//    seg.setMethodType (pcl::SAC_RANSAC);
//    seg.setMaxIterations (maxIterations);
//    seg.setDistanceThreshold (distanceThreshold);
//
//    seg.setInputCloud(cloud);
//    seg.segment(*inliers, *coefficients);  //dereference.
    auto inliers = Ransac<PointT>(cloud, maxIterations, distanceThreshold);

    if (inliers->indices.empty())
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }



    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
//    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//    tree->setInputCloud (cloud);
    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;

    int index = 0;
    for (const auto &point: cloud->points){
        std::vector<float> new_point{point.data[0], point.data[1], point.data[2]};
        points.emplace_back(new_point);
        tree->insert(new_point, index, 3);
        index++;
    }

    std::vector<pcl::PointIndices> cluster_indices;
//    pcl::EuclideanClusterExtraction<PointT> ec;
//    ec.setClusterTolerance (clusterTolerance); // 2cm
//    ec.setMinClusterSize (minSize);
//    ec.setMaxClusterSize (maxSize);
//    ec.setSearchMethod (tree);
//    ec.setInputCloud (cloud);
//    ec.extract (cluster_indices);


    auto indices_vector = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);


    for(const auto &indices: indices_vector){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (const auto &i: indices){
            cloud_cluster->points.emplace_back(cloud->points.at(i));
        }
        cloud_cluster->width = indices.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        clusters.template emplace_back(cloud_cluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}