// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    //Self-implemented RANSAC
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Segment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    //Self-implemented Clustering algorithm
    //Helper function for clustering
    void fillClusterMine(int index, const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, std::vector<int> & cluster, std::vector<int> & processed)
    {
        //Add the point to the vector and marked as proccessed
        processed.push_back(index);
        cluster.push_back(index);

        //Find the nearby point
        std::vector<int> neighbours = tree->search(points[index],distanceTol);
        //Loop through the point recursively until all eligible and not owned points is in this cluster
        for (auto near_iter = neighbours.begin(); near_iter != neighbours.end();++near_iter){
            int idx = *near_iter;
            if (std::find(processed.begin(),processed.end(),idx) == processed.end())
                fillClusterMine(*near_iter, points, tree, distanceTol, cluster, processed);
        }
    }

    std::vector<typename pcl::PointCloud<PointT>::Ptr> ClusteringXYZ(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize){
        // Time segmentation process
        auto startTime = std::chrono::steady_clock::now();
        
        //Build our own KdTree
        KdTree* tree = new KdTree;
        std::vector<std::vector<float>> points;
        for (int i = 0 ; i < cloud->size(); ++i){
            //Create a vector with the point
            std::vector<float> aVect = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
            //Insert into the tree per normal
            tree->insert(aVect, i);
            points.push_back(aVect);
        }

        std::cout << "Done constructing KD Tree" << std::endl;

        //Setup before doing some clustering
        std::vector<int> processed;
        //Return a vector of pointers to pointcloud
        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        //Loop through the points in the points
        for (auto j = 0; j < points.size() ; ++j){
        //If not already processed

            if (std::find(processed.begin(), processed.end(), j)!= processed.end())
                continue;

            else{
                // Create a new cluster
                std::vector<int> cluster;
                // Fill the cluster
                fillClusterMine(j, points, tree, clusterTolerance, cluster, processed);
                //Push the cluster back into the list of clusters
                // Create the filtering object
                pcl::ExtractIndices<PointT> extract;
                pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
                inliers->indices = cluster; 

                //Filtering largest plane inliers out of the point cloud
                extract.setInputCloud(cloud);
                extract.setIndices(inliers);

                //Selecting all points in pointcloud whose are inliers
                extract.setNegative(false);

                typename pcl::PointCloud<PointT>::Ptr filledCluster(new pcl::PointCloud<PointT>);
                extract.filter(*filledCluster);
                if (filledCluster->size() >= minSize && filledCluster->size() <= maxSize)
                    clusters.push_back(filledCluster);
            }
        
        }

        //Turn off timer and calculate time elapsed
        auto endTime = std::chrono::steady_clock::now();
        auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
        std::cout << "clustering found " << clusters.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;
        return clusters;
    }

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};
#endif /* PROCESSPOINTCLOUDS_H_ */