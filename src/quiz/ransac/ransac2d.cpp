/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}
std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function

    int size_p = cloud->points.size();
    // For max iterations
    int max_num_inliers = 0;
    std::unordered_set<int> inliersTemp;

    for(auto i = 0; i < maxIterations; i++){
        int num_inliers = 0;

        // Randomly sample subset and fit line
        int r_1 = rand() % (size_p - 0) + 0; // 取得[a,b)的随机整数，使用(rand() % (b-a))+ a （结果值含a不含b）。
        int r_2 = rand() % (size_p - 0) + 0;
        auto p_1 = cloud->points.at(r_1);
        auto p_2 = cloud->points.at(r_2);
        double A = p_1.y - p_2.y;
        double B = p_2.x - p_1.x;
        double C = (p_1.x * p_2.y - p_2.x * p_1.y);
//        (y1 -y2)x + (x2 -x1)y + (x1*y2 -x2*y1) = 0;
//         Ax + By + C = 0

        // Measure distance between every point and fitted line  d = |Ax+By+C|/sqrt(A^2+B^2)
        for(int j = 0; j <  cloud->points.size(); j++){
            auto point = cloud->points.at(j);
            double d = std::abs(A * point.x + B * point.y + C) / std::sqrt(A * A + B * B);
            if(d < distanceTol){
                num_inliers ++;
                inliersTemp.emplace(j);
            }
        }
        // If distance is smaller than threshold count it as inlier

        if(num_inliers > max_num_inliers){
            max_num_inliers = num_inliers;
            inliersResult.swap(inliersTemp);
            inliersTemp.clear();
        }
    }


    // Return indicies of inliers from fitted line with most inliers

    return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
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



	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();


	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	std::cout << "inliers size is: " << inliers.size() << std::endl;
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}

  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}

}
