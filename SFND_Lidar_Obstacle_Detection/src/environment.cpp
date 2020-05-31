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

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    
    // Render flags
    bool renderCluster = true;
    bool renderBoundingBox = true;

    // Filter hyper parameters
    float filterRes = 0.15; //0.3
    int maxIT = 50; //40
    float planeRes = 0.2; //0.3

    // kD Cluster hyper parameters
    float clusterTolerance = 0.3; //0.5
    int minSize = 10; //50
    int maxSize = 1000; //140

    /*  ####################################################################################################################### */
    
    /* ################# FILTERING ######################################## */

    // Min and Max values for CropBox region of interest based filtering
    // Eigen::Vector4f minPoint (-10.0, -6.5, -3.0, 1.0); //-3.0-third
    // Eigen::Vector4f maxPoint (30.0, 6.0, 1.0, 1.0); // Removed ground plane obstacles and buildings on the side.
    Eigen::Vector4f minPoint (-20.0, -6.0, -2.0, 1.0); //-3.0-third
    Eigen::Vector4f maxPoint (30.0, 7.0, 5.0, 1.0); // Removed ground plane obstacles and buildings on the side.
    pcl::PointCloud<pcl::PointXYZI>::Ptr boxFilteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);
    // renderPointCloud(viewer,boxFilteredCloud,"boxFilteredCloud");

    /* ################# SEGMENTATION ######################################## */

    // Plane Segmentation 
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(boxFilteredCloud, 100, 0.2);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->RansacPlane(boxFilteredCloud, maxIT, planeRes);
    // renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1,0,0)); // Red
    renderPointCloud(viewer, segmentCloud.second, "Road Cloud", Color(1,1,0)); // Yellow

    /* ################# EUCLEDIAN CLUSTERING ######################################## */

    // Clustering
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->Clustering(segmentCloud.first, 1.0, 20, 1000);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->KdTreeClustering(segmentCloud.first, clusterTolerance, minSize, maxSize);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(0,1,0), Color(1,1,1), Color(0,0,1)}; // Green, White, Blue
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        if (renderCluster)
        {
            std::cout << "cluster size ";
            pointProcessorI->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        if (renderBoundingBox)
        {
            Box box = pointProcessorI->BoundingBox(cluster);
            renderBox(viewer,box,clusterId, Color(1,0,0),1);
        }
        
        ++clusterId;  

    }
    
    
    // // Render a box to represent the ego car
    // Box box = pointProcessorI->BoundingBox();
    // renderBox(viewer,box,clusterId);
    
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    bool renderCluster = true;
    bool renderBoundingBox = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidarCloud = lidar->scan();
    // renderRays(viewer, lidar->position , lidarCloud);
    renderPointCloud(viewer, lidarCloud, "LiDAR PCD", Color(0,1,0));
    // TODO:: Create point processor

    ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();

    // Plane Segmentation 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(lidarCloud, 100, 0.2);
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->RansacPlane(lidarCloud, 300, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "Obstacle Cloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "Road Cloud", Color(0,0,1));

    // Clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(1,0,1)};
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        if (renderCluster)
        {
            std::cout << "cluster size ";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        }
        if (renderBoundingBox)
        {
            Box box = pointProcessor->BoundingBox(cluster);
            renderBox(viewer,box,clusterId);
        }
        
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    
    // Simple highway
    // simpleHighway(viewer);

    // Real City block
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    // std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_2");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
        
    } 
}