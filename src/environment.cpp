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
    
    // TODO:: Create lidar sensor 
    // Lidar * old_ptr = new Lidar(cars, 0);
    std::shared_ptr<Lidar> lidar_ptr(new Lidar(cars, 0));

    // get scanned point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr = lidar_ptr->scan();

    // display
    // renderRays(viewer, lidar_ptr->position, pc_ptr);
    // std::string test("simulated point cloud");
    // renderPointCloud(viewer, pc_ptr, test);


    // TODO:: Create point processor
    // ProcessPointClouds<pcl::PointXYZ> processor;
    // std::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> processor_ptr; // [NOTE] this is nullptr but still works.
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZ>> processor_ptr(new ProcessPointClouds<pcl::PointXYZ>());
    if (processor_ptr == nullptr)
    {
        std::cout << " processor_ptr == nullptr \n";
    }else{
        std::cout << " processor_ptr is valid.\n";
    }
    

    // seperate ground.
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = processor_ptr->SegmentPlane(pc_ptr, maxIterations, distanceThreshold);
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(1,1,1));

    // cluster non-ground pts.
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor_ptr->Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    
    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        // render clusters.
        std::cout << "cluster size ";
        processor_ptr->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);

        // render bbox.
        Box box = processor_ptr->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{

  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    // // create processor.
    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI(new ProcessPointClouds<pcl::PointXYZI>());

    // read point cloud from file.
    const pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("src/sensors/data/pcd/data_1/0000000000.pcd");
    // renderPointCloud(viewer,inputCloud,"inputCloud");

    // downsample and crop points in ROI.
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-10, -7, -2, 1), Eigen::Vector4f ( 50, 7, 10, 1));
    // renderPointCloud(viewer,filterCloud,"filterCloud");
  
    // seperate ground.
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, maxIterations, distanceThreshold);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->MySegmentPlaneRansac3D(filterCloud, maxIterations, distanceThreshold);

    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,1,1));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // cluster non-ground pts.
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->MyClustering(segmentCloud.first, 0.5, 3, 3000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // // print clusters information.
        // std::cout << "cluster size ";
        // pointProcessorI->numPoints(cluster);

        // // render clusters.
        // renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);

        // render bbox.
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{

  // ----------------------------------------------------
  // -----Open 3D viewer and display City Block     -----
  // ----------------------------------------------------

    // downsample and crop points in ROI.
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.2 , Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f ( 30, 8, 1, 1));
  
    // seperate ground.
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->MySegmentPlaneRansac3D(filterCloud, maxIterations, distanceThreshold);
    
    renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,1,1));
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    // cluster non-ground pts.
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->MyClustering(segmentCloud.first, 0.45, 5, 3000);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // // render clusters.
        // renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%2]);

        // render bbox.
        Box box = pointProcessorI->BoundingBox(cluster);
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


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    // simpleHighway(viewer);

    // while (!viewer->wasStopped ())
    // {
    //     viewer->spinOnce ();
    // } 


    std::shared_ptr<ProcessPointClouds<pcl::PointXYZI>> pointProcessorI(new ProcessPointClouds<pcl::PointXYZI>());
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
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
        if(streamIterator == stream.end()) { streamIterator = stream.begin(); }

        viewer->spinOnce ();
    }

}