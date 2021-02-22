/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // Create lidar sensor with the set of cars, groudslope set to 0
    Lidar * lidarSensor = new Lidar(cars, 0);
    // Create point cloud and do the raycasting with the lidar object
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudGenerated = lidarSensor->scan();
    //Call The render function
    //renderRays(viewer,lidarSensor->position,pointCloudGenerated);
    renderPointCloud(viewer,pointCloudGenerated,"rendered");

    //Create a point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    //Process and render the planar and obstacle components
    auto segmentedPair = pointProcessor.SegmentPlane(pointCloudGenerated,100,0.2);
    renderPointCloud(viewer,segmentedPair.first,"obstacles",Color(1,0,0));
    renderPointCloud(viewer,segmentedPair.second,"segmented plane",Color(0,1,0));

    //Cluster the obstacles together
    //Get a list of the cluster point clouds based on the obstacles pcd
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentedPair.first, 2.0, 3, 30);

    //Prepare a set of colour
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    //Loop and render each extracted/clustered point clouds with the corresponding color
    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters){
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer){
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------

    // Create a point processor (maybe on a heap this time):
    ProcessPointClouds<pcl::PointXYZI> * pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>;
    // Load a point cloud and render it
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    
    //Parameters
    float leafSize = 1.0f;
    // 1st: 2nd: Width of road 3rd: Height
    Eigen::Vector4f minPt(-11.0,-6.5, -3.8, 1.0);
    Eigen::Vector4f maxPt(20.0, 7.5, 10.0, 1.0);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.1, minPt, maxPt);
    
    //renderPointCloud(viewer,filteredCloud,"inputCloud");

    //For DEBUG and TESTING
    // Box box = pointProcessorI->BoundingBox(filteredCloud);
    // renderBox(viewer,box,999);

    //Process and render the planar and obstacle components
    auto segmentedPair = pointProcessorI->SegmentPlane(filteredCloud,100,0.2);
    renderPointCloud(viewer,segmentedPair.first,"obstacles",Color(1,0,0));
    renderPointCloud(viewer,segmentedPair.second,"segmented plane",Color(0,1,0));

    //Cluster the obstacles together
    //Get a list of the cluster point clouds based on the obstacles pcd
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentedPair.first, 0.4, 50, 1500);

    //Prepare a set of colour
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0),Color(1,1,0),Color(0,0,1)};

    //Loop and render each extracted/clustered point clouds with the corresponding color
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters){
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }


}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}