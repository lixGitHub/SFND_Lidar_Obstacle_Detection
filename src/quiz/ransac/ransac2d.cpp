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


std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	int idx1, idx2, idx3;
	int start = 0, end = cloud->points.size()-1;
	int max_inlier_count = 0;
	double best_a, best_b, best_c, best_w, best_d;
	double temp_dist = 0;

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset to construct a plane
		// chose index.
		idx1 = start + rand() % (end - start + 1);
    	idx2 = start + rand() % (end - start + 1);
		idx3 = start + rand() % (end - start + 1);
		// Ensure the two indices are unique
		while (idx1 == idx2 || idx1 == idx3 || idx2 == idx3) {
			idx2 = start + rand() % (end - start + 1);
			idx3 = start + rand() % (end - start + 1);
		}

		double x1 = cloud->points[idx1].x;
		double y1 = cloud->points[idx1].y;
		double z1 = cloud->points[idx1].z;
		double x2 = cloud->points[idx2].x;
		double y2 = cloud->points[idx2].y;
		double z2 = cloud->points[idx2].z;
		double x3 = cloud->points[idx3].x;
		double y3 = cloud->points[idx3].y;
		double z3 = cloud->points[idx3].z;

		double a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		double b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		double c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		double w = sqrt(a*a + b*b + c*c);
		double d = -(a * x1 + b * y1 + c * z1);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		int inlier_count = 0;
		for (int j = 0; j < cloud->points.size(); j++)
		{
			temp_dist = abs(a*cloud->points[j].x + b*cloud->points[j].y + c*cloud->points[j].z + d) / w ;
			if (temp_dist < distanceTol)
			{
				inlier_count++;
			}
		}
		if (inlier_count > max_inlier_count){
			max_inlier_count = inlier_count;
			best_a = a;
			best_b = b;
			best_c = c;
			best_d = d;
			best_w = w;
		}
		// std::cout << "iteration: " << i << "  inlier count: " << inlier_count << std::endl;
	}
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		temp_dist = abs(best_a*cloud->points[i].x + best_b*cloud->points[i].y + best_c*cloud->points[i].z + best_d) / best_w ;
		if (temp_dist < distanceTol)
		{
			inliersResult.insert(i);
		}
	}
	// std::cout <<  " max inlier count: " << inliersResult.size() << std::endl;

	// Return indicies of inliers from fitted line with most inliers
	return inliersResult;
}


std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	int idx1, idx2;
	int start = 0, end = cloud->points.size()-1;
	int max_inlier_count = 0;
	double best_a, best_b, best_c, best_w;
	double temp_dist = 0;

	// For max iterations 
	for (int i = 0; i < maxIterations; i++)
	{
		// Randomly sample subset and fit line
		// chose index.
		idx1 = start + rand() % (end - start + 1);
    	idx2 = start + rand() % (end - start + 1);
		// Ensure the two indices are unique
		while (idx1 == idx2) {
			idx2 = start + rand() % (end - start + 1);
		}

		double x1 = cloud->points[idx1].x;
		double y1 = cloud->points[idx1].y;
		double x2 = cloud->points[idx2].x;
		double y2 = cloud->points[idx2].y;

		double a = y1 - y2;
		double b = x2 - x1;
		double c = x1*y2 - x2*y1;
		double w = sqrt(a*a + b*b);

		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		int inlier_count = 0;
		for (int j = 0; j < cloud->points.size(); j++)
		{
			temp_dist = abs(a*cloud->points[j].x + b*cloud->points[j].y+c) / w ;
			if (temp_dist < distanceTol)
			{
				inlier_count++;
			}
		}
		if (inlier_count > max_inlier_count){
			max_inlier_count = inlier_count;
			best_a = a;
			best_b = b;
			best_c = c;
			best_w = w;
		}
	}
	
	for (int i = 0; i < cloud->points.size(); i++)
	{
		temp_dist = abs(best_a*cloud->points[i].x + best_b*cloud->points[i].y + best_c) / best_w ;
		if (temp_dist < distanceTol)
		{
			inliersResult.insert(i);
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
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = Ransac(cloud, 10, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, cloud->points.size() * 0.1, 0.3);

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
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(1,0,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,1,1));
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
