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

/*
Input for this function:
- ptr to point cloud
- filter resolution (float) -> leaf size
- minPoint and maxPoint that define the region of interest
*/
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // First, do voxel fitering to reduce the number of points to a voxel centroid
    pcl::VoxelGrid<PointT> voxFilter;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    voxFilter.setInputCloud(cloud);
    voxFilter.setLeafSize (filterRes, filterRes, filterRes);
    voxFilter.filter(*cloudFiltered);

    // Then do region of interest stuffs
    // For simplicity sake, use a static cropbox approach to cut out irrelevant point
    // A smarter way of RoI can be applied by using SMIRE

    typename pcl::PointCloud<PointT>::Ptr cloudFinalRegion(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roadBox(true);
    roadBox.setMin(minPoint);
    roadBox.setMax(maxPoint);
    roadBox.setInputCloud(cloudFiltered);
    roadBox.filter(*cloudFinalRegion);

    //Filter the roof out
    pcl::CropBox<PointT> roofBox(true);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // Those that are inside the ROI but not roof
    //Hard coded
    roofBox.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    roofBox.setMax(Eigen::Vector4f(2.6,1.7,-.4,1));
    roofBox.setInputCloud(cloudFinalRegion);
    roofBox.filter(inliers->indices);

    //Extract the roof indices - inliers from the point cloud out
    pcl::ExtractIndices<PointT> extract; //Extractor
    extract.setInputCloud(cloudFinalRegion);
    extract.setIndices(inliers);
    extract.setNegative(true); // To minus away the inliers
    extract.filter(*cloudFinalRegion); // Inplace filter out

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFinalRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  //Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr segmentedPlane(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
    
    // Create the filtering object
    pcl::ExtractIndices<PointT> extract;

    //Filtering largest plane inliers out of the point cloud
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);

    //Selecting all points in pointcloud whose are inliers
    extract.setNegative(false);
    extract.filter(*segmentedPlane);
    std::cerr << "PointCloud representing the planar component: " << segmentedPlane->width * segmentedPlane->height << " data points." << std::endl;

    //Selecting all points that are outliers (not the planar components)
    extract.setNegative(true);
    extract.filter(*obstacles);

    //Return obstacle then segmented
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, segmentedPlane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    //Will this even works if on stack?
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    //Create Segmentation object (In charge of segmenting the plane with RANSAC of SAC)
    pcl::SACSegmentation<PointT> seg;

    seg.setModelType(pcl::SACMODEL_PLANE); // For plane extraction
    seg.setMethodType(pcl::SAC_RANSAC); //SAC method is Random sample consensus
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment (*inliers, *coefficients); // Pass pointers by reference by dereference them
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }
    // Filter and extract the inliers of the cloud (those that belongs to the same surface)
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    //Since road is the big flat surface, we should end up with separating it from the rest of the objects
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::cout << "Using our own RANSAC..." << std::endl;
    //Timing
    auto startTime = std::chrono::steady_clock::now();

    //Set seed and to store result
    std::unordered_set<int> inliersResult;
	srand(time(NULL));

    //RANSAC implementation

	while (maxIterations--){
		//Create an inlier set that is initially empty (prevent replication in population of 3 points)
		std::unordered_set<int> inliers;
		
		//Populate the unoderded set with 3 inliers index from the cloud data
		while(inliers.size()<3){
			inliers.insert(rand()%(cloud->points.size()));
		}

		//Need this for plane equation
		float x1, y1, z1; 
		float x2, y2, z2; 
		float x3, y3, z3;

		//Random sample 3 points for a plane
		auto it = inliers.begin();
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;
		z1 = cloud->points[*it].z;

		++it;

		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;
		z2 = cloud->points[*it].z;

		++it;

		x3 = cloud->points[*it].x;
		y3 = cloud->points[*it].y;
		z3 = cloud->points[*it].z;

		// Coefficient for Plane and distance
		float i, j, k;
		float A, B, C, D;

		i = (y2-y1)*(z3-z1) - (z2-z1)*(y3-y1);
		j = (z2-z1)*(x3-x1) - (x2-x1)*(z3-z1);
		k = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);

		A = i;
		B = j;
		C = k;
		D = - (i*x1 + j*y1 + k*z1);

		//Loop through all the points in point cloud
		for (auto itPt = cloud->points.begin(); itPt != cloud->points.end(); ++itPt){

			float distance;
			float x,y,z;

			//Check if the point is the one we have already included
			//First calculate the index of point being iterate
			//Check if it is already in the inliers group
			if (inliers.count(itPt - cloud->points.begin())){
				//Already inside inliers, skipping
				continue;
			}

			//Extract cordinates and calculate distance
			x = itPt->x;
			y = itPt->y;
			z = itPt->z;

			//Distance
			distance = fabs(A*x+B*y+C*z+D)/sqrt(pow(A,2)+pow(B,2)+pow(C,2));

			//Check distance and tolerance and add to inliers
			if (distance <= distanceThreshold){
				inliers.insert(itPt - cloud->points.begin());
			}
		}
		//Check if this iterations has more inliers than the previous ones and assign to the results
		if (inliers.size() >= inliersResult.size())
			inliersResult = inliers;
	}

    // Filter and extract the inliers of the cloud (those that belongs to the same surface)
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    //Since road is the big flat surface, we should end up with separating it from the rest of the objects
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;

}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    //For returning to calling function an array of clustered pointers
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    
    //To speed up, build a tree for the cluster
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    //Set up indices objects to store those belongs to a cluster
    std::vector<pcl::PointIndices> cluster_indices;
    //Set up an object to do clustering
    pcl::EuclideanClusterExtraction<PointT> ec;
    //Init its parameters
    //Any point within the tolerance will be group together
    //too small then you would try to group bigger or just discard
    //too big clusters means we need to break it down smaller
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    //Last argument is the KdTree
    ec.setSearchMethod(tree);

    //Extract the cluster out
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    //Create point clouds and store in the returning array of point cloud
    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            cloud_cluster->push_back ((*cloud)[*pit]); //*
            cloud_cluster->width = cloud_cluster->size ();
            cloud_cluster->height = 1;
            cloud_cluster->is_dense = true;

            std::cout << "PointCloud " << j << " representing the Cluster: " << cloud_cluster->size () << " data points." << std::endl;
            clusters.push_back(cloud_cluster);
            j++;
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