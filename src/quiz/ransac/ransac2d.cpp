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
	return pointProcessor.loadPcd("/home/boom/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/simpleHighway.pcd");
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

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers

	while (maxIterations--){
		//Create an inlier set that is initially empty (prevent replication in population of 2 points)
		std::unordered_set<int> inliers;
		
		//Populate the unoderded set with 2 inliers index from the cloud
		while(inliers.size()<2){
			inliers.insert(rand()%(cloud->points.size()));
		}

		//Then we get x1, y1, x2, y2, etc for line equation
		float x1, y1, x2, y2;

		//Should get the 2 point that form the line
		auto it = inliers.begin();
		x1 = cloud->points[*it].x;
		y1 = cloud->points[*it].y;

		++it;

		x2 = cloud->points[*it].x;
		y2 = cloud->points[*it].y;

		// Get the line equation coefficient terms
		float A, B, C;
		A = y1 - y2;
		B = x2 - x1;
		C = x1*y2 - x2*y1;

		//Loop through all the points

		for (auto itPt = cloud->points.begin(); itPt != cloud->points.end(); ++itPt){

			float distance;
			float x,y;

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

			//Distance
			distance = fabs(A*x+B*y+C)/sqrt(pow(A,2)+pow(B,2));

			//Check distance and tolerance and add to inliers
			if (distance <= distanceTol){
				inliers.insert(itPt - cloud->points.begin());
			}
		}
		//Check if this iterations has more inliers than the previous ones and assign to the results
		if (inliers.size() >= inliersResult.size())
			inliersResult = inliers;
	}
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

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
			if (distance <= distanceTol){
				inliers.insert(itPt - cloud->points.begin());
			}
		}
		//Check if this iterations has more inliers than the previous ones and assign to the results
		if (inliers.size() >= inliersResult.size())
			inliersResult = inliers;
	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = RansacPlane(cloud, 100 , 0.2);

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


