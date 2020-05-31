// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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

    typename pcl::PointCloud<PointT>::Ptr filteredCloud { new (pcl::PointCloud<PointT>) };
    typename pcl::PointCloud<PointT>::Ptr boxFilteredCloud { new (pcl::PointCloud<PointT>) };

    // Create the filtering object
    typename pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*filteredCloud);
    
    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::CropBox<PointT> boxFilter(true);
    boxFilter.setMin (minPoint);
    boxFilter.setMax (maxPoint);
    boxFilter.setInputCloud (filteredCloud);
    boxFilter.filter (*boxFilteredCloud);

    // To remove static roof points
    std::vector<int> indices;
    
    pcl::CropBox<PointT> roof(true);
    roof.setMin (Eigen::Vector4f (-1.5, -1.7, -1, 1.0));
    roof.setMax (Eigen::Vector4f (2.6, 1.7, -0.4, 1.0));
    roof.setInputCloud (boxFilteredCloud);
    roof.filter (indices);

    pcl::PointIndices::Ptr inliers { new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (boxFilteredCloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*boxFilteredCloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << " Filtered cloud size: " << boxFilteredCloud->points.size() << std::endl;


    return boxFilteredCloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud { new (pcl::PointCloud<PointT>) };
    typename pcl::PointCloud<PointT>::Ptr roadCloud { new (pcl::PointCloud<PointT>) };

    for (int index : inliers->indices)
    {
        roadCloud->points.push_back(cloud->points[index]); // Same result as implemented on the comment below for Road.
    }

    // Create the filtering object
    // pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::ExtractIndices<PointT> extract;

    // Extract the inliers (Road)
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    // extract.setNegative (false);
    // extract.filter (*roadCloud);

    // Extract the inliers (Car obstacles)
    extract.setNegative (true);
    extract.filter (*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, roadCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};
    
    // Create the segmentation object
    // pcl::SACSegmentation<pcl::PointXYZ> seg;
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

    if (inliers->indices.size() == 0)
    {
        std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	PointT point1;
	PointT point2;
	PointT point3;
	int idx1;
	int idx2;
	int idx3;
	float a,b,c,d,dis,len;

	// For max iterations
	for(int it=0;it<maxIterations;it++)
	{
		/*Temporary buffer to hold identified points in current loop*/
		std::unordered_set<int> tempIndices;
		/*Identify 3 points randomly*/
		while(tempIndices.size()<3)
			tempIndices.insert((rand() % cloud->points.size()));
		auto iter = tempIndices.begin();
		idx1 = *iter;
		++iter;
		idx2 = *iter;
		++iter;
		idx3 = *iter;

		point1 = cloud->points[idx1];
		point2 = cloud->points[idx2];
		point3 = cloud->points[idx3];

		/*Fit a plane using the above 3 points*/
		a = (((point2.y-point1.y)*(point3.z-point1.z))-((point2.z-point1.z)*(point3.y-point1.y)));
		b = (((point2.z-point1.z)*(point3.x-point1.x))-((point2.x-point1.x)*(point3.z-point1.z)));
		c = (((point2.x-point1.x)*(point3.y-point1.y))-((point2.y-point1.y)*(point3.x-point1.x)));
		d = -(a*point1.x+b*point1.y+c*point1.z);
		len = sqrt(a*a+b*b+c*c);

		// Measure distance between every point and fitted plane
		for(int pt_cnt=0;pt_cnt<cloud->points.size();pt_cnt++)
		{
			if(pt_cnt!=idx1||pt_cnt!=idx2||pt_cnt!=idx3)
			{
				dis = (fabs(a*cloud->points[pt_cnt].x+b*cloud->points[pt_cnt].y+c*cloud->points[pt_cnt].z+d)/len);
				// If distance is smaller than threshold count it as inlier
				if(dis<=distanceThreshold)
				{
					tempIndices.insert(pt_cnt);
				}
			}
		}

		/*Store the temporary buffer if the size if more than previously idenfitied points */
		if(tempIndices.size()>inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = tempIndices;

		}

	}

	// Segment the largest planar component from the remaining cloud
	if (inliersResult.size () == 0)
	{
	  std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	}
	/*Buffers to hold cloud and object points*/
	typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	/*Copy the points from inputcloud in to cloudInliers if the indices is in inliersResult vector
	 * or else copy the point to cloudOutliers*/
	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
	/*Create a pair using inlier and outlier points*/
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;


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
    // pcl::search::kdTree<pcl::PointXYZ>::Ptr tree { new pcl::search::kdTree<pcl::PointXYZ>)};
    typename pcl::search::KdTree<PointT>::Ptr tree ( new pcl::search::KdTree<PointT> );
    tree->setInputCloud (cloud);
    
    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); 
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    // Get cluster cloud
    for (pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int i: getIndices.indices)
            cloudCluster->points.push_back(cloud->points[i]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper (int id, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, typename KdTree<PointT>::KdTree* tree, float distanceTol, int maxSize)
{
	if (processed[id] == false && cluster.size() <= maxSize)
    {
        processed[id] = true;
        cluster.push_back(id);

        // Nearby points
        // std::vector<int> nearby = tree->search(points[id], distanceTol);
        std::vector<int> nearby = tree->search(cloud->points[id], distanceTol);

        for (int i : nearby)
        {
            if (!processed[i])
                clusterHelper(i, cloud, cluster, processed, tree, distanceTol, maxSize);
        }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud,  typename KdTree<PointT>::KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	
	// std::vector<bool> processed(points.size(), false);
	std::vector<bool> processed(cloud->points.size(), false);
    int i = 0;
    while (i < cloud->points.size())
    {
        if (processed[i])
        {
            i++;
            continue;
        }
        std::vector<int> cluster;
        // clusterHelper (i, points, cluster, processed, tree, distanceTol);
        clusterHelper (i, cloud, cluster, processed, tree, distanceTol, maxSize);

        // Check to see if the clusters are between the provided limits
		if (cluster.size() >= minSize && cluster.size() <= maxSize)
        {
            clusters.push_back(cluster);
		    i++;
        }
    }

	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::KdTreeClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    
    // Custom kDTree
    // KdTree* tree = new KdTree;
    typename KdTree<PointT>::KdTree *tree = new KdTree<PointT>;
    tree->insert(cloud);
    // for (int i = 0; i < cloud->points.size(); i++)
    // {
    //     tree->insert(cloud->points[i], i);
    // }
    
    // Cluster
    std::vector<std::vector<int>> clusterIdx =  ProcessPointClouds<PointT>::euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);
    
    // for (auto clusterIdx : clusterIdx)
    for (auto Idx : clusterIdx)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud (new pcl::PointCloud<PointT>());
        // for (int indices : clusterIdx)
        for (int indices : Idx)
        {
            clusterCloud->points.push_back(cloud->points[indices]);
        }
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;
        // if (clusterCloud->width >= minSize && clusterCloud->width <= maxSize)
        clusters.push_back(clusterCloud);

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