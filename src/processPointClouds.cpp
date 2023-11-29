// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <iomanip>

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

    // downsample.
    typename pcl::PointCloud<PointT>::Ptr cloud_downsample(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_downsample);

    // crop the ROI.
    typename pcl::PointCloud<PointT>::Ptr cloud_crop(new pcl::PointCloud<PointT>());
    pcl::CropBox<PointT> crop(true);
    crop.setMin(minPoint);
    crop.setMax(maxPoint);
    crop.setInputCloud(cloud_downsample);
    crop.filter(*cloud_crop);


    // remove the ego car
    typename pcl::PointCloud<PointT>::Ptr cloud_remove_ego(new pcl::PointCloud<PointT>());
    crop.setInputCloud(cloud_crop);
    crop.setMin(Eigen::Vector4f(-1.5,-1.7,-1, 1));
    crop.setMax(Eigen::Vector4f( 2.6, 1.7, -0.4, 1));
    crop.filter(*cloud_remove_ego);

    // [option 2] 
    // std::vector<int> index;
    // crop.filter(index);

    // pcl::PointIndices outer_indices;
    pcl::PointIndices::Ptr outer_indices_ptr (new pcl::PointIndices ());
    crop.getRemovedIndices(*outer_indices_ptr);

    // Extract the outerlier, the non-ego points.
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud_crop);
    extract.setIndices (outer_indices_ptr);
    extract.setNegative (false);
    // typename pcl::PointCloud<PointT>::Ptr cloud_outer(new pcl::PointCloud<PointT>());
    extract.filter (*cloud_remove_ego);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_remove_ego;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

    typename pcl::PointCloud<PointT>::Ptr cloud_p(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_f(new pcl::PointCloud<PointT>());

    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    // std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

    // Create the filtering object
    extract.setNegative (true);
    extract.filter (*cloud_f);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_f, cloud_p);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	// pcl::PointIndices::Ptr inliers;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // TODO:: Fill in this function to find inliers for the cloud.

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }else{
        std::cout << "ground pts inlier: " << inliers->indices.size() << "\n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


// Ransac3D based ground segmentation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MySegmentPlaneRansac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

	srand(time(NULL));
	
	// TODO: Fill in this function

	int idx1, idx2, idx3;
	int start = 0, end = cloud->points.size()-1;
	int max_inlier_count = 0;
    float x1, y1, z1, x2, y2, z2, x3, y3, z3, a, b, c, d, w;
	float best_a, best_b, best_c, best_w, best_d;
	float temp_dist = 0;

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

		x1 = cloud->points[idx1].x;
		y1 = cloud->points[idx1].y;
		z1 = cloud->points[idx1].z;
		x2 = cloud->points[idx2].x;
		y2 = cloud->points[idx2].y;
		z2 = cloud->points[idx2].z;
		x3 = cloud->points[idx3].x;
		y3 = cloud->points[idx3].y;
		z3 = cloud->points[idx3].z;

		a = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		b = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
		w = sqrt(a*a + b*b + c*c);
		d = -(a * x1 + b * y1 + c * z1);

		// Measure distance between every point and fitted plane
		// If distance is smaller than threshold count it as inlier
		int inlier_count = 0;
		for (int j = 0; j < cloud->points.size(); j++)
		{
			temp_dist = abs(a*cloud->points[j].x + b*cloud->points[j].y + c*cloud->points[j].z + d) / w ;
			if (temp_dist < distanceThreshold)
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

    typename pcl::PointCloud<PointT>::Ptr cloud_g(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloud_ng(new pcl::PointCloud<PointT>());

	for(int i = 0; i < cloud->points.size(); i++)
	{
        PointT point = cloud->points[i];
        temp_dist = abs(best_a*point.x + best_b*point.y + best_c*point.z + best_d) / best_w ;
		if (temp_dist < distanceThreshold) { cloud_g->points.push_back(point); }
        else { cloud_ng->points.push_back(point); }
	}
    // std::cout <<  " ground pts count: " << cloud_g->points.size() << "  non-ground pts count: " << cloud_ng->points.size() << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	// Return ground and nonground points.
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_ng, cloud_g);
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
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
  
    // cluster
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // set result.
    int j = 0;
    for (const auto& cluster : cluster_indices)
    {
      typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>());
      for (const auto& idx : cluster.indices) {
        cloud_cluster->push_back((*cloud)[idx]);
      } //*
      cloud_cluster->width = cloud_cluster->size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true; // ?
  
    //   std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;

      j++;
      clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::MyClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // Creating the KdTree object for the search method of the extraction
    std::shared_ptr<KdTree3D> kdtree(new KdTree3D);
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> test{cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        kdtree->InsertPoint(test, i);
    }

    // Use kdtree to do euclidean cluster. save the point index of each cluster.
	std::vector<std::vector<int>> clusters_ids;

	std::vector<bool> visited(cloud->points.size(), false);
	for (int i = 0; i < cloud->points.size(); i++)
	{
		if (!visited[i])
		{
			// initialize a empty cluster.
			std::vector<int> cluster;
			// set the seed.
			std::queue<int> expand_list;
            visited[i] = true;
			expand_list.push(i);
			// find all points of the cluster.
			while (!expand_list.empty())
			{
				int front_id = expand_list.front();
				expand_list.pop();
				cluster.push_back(front_id);

				std::vector<int> nearbys = kdtree->SearchNeighbour(std::vector<float>{cloud->points[front_id].x, cloud->points[front_id].y, cloud->points[front_id].z}, clusterTolerance);
                
				for (int id: nearbys){ 
					if (!visited[id])
					{
                        visited[id] = true;
						expand_list.push(id); 
					}
				}
			}
            clusters_ids.push_back(cluster);
		}
	}

    // save the points of each cluster.
    for (const auto& cluster : clusters_ids)
    {
        if (cluster.size() < minSize) { continue; }

        typename pcl::PointCloud<PointT>::Ptr one_cluster (new pcl::PointCloud<PointT>());
        for (const auto& idx : cluster) {
            one_cluster->push_back((*cloud)[idx]);
        }
        one_cluster->width = one_cluster->size ();
        one_cluster->height = 1;
        one_cluster->is_dense = true; // ?

        clusters.push_back(one_cluster);
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
