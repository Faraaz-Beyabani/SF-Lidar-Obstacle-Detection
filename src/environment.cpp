/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
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
  
  Lidar* lidar = new Lidar(cars, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud = lidar->scan();

  ProcessPointClouds<pcl::PointXYZ> ppc;
  std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = ppc.SegmentPlane(pCloud, 1, 0.2);
  renderPointCloud(viewer, segmentCloud.first, "obst", Color(1, 0, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = ppc.Clustering(segmentCloud.first, 1.0, 3, 50);

  int clusterID = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

  for(auto cluster : clusters)
  {
    renderPointCloud(viewer, cluster, "obstacle"+clusterID, colors[clusterID%colors.size()]);
    Box clusterBox = ppc.BoundingBox(cluster);
    renderBox(viewer, clusterBox, clusterID);
    clusterID++;
  } 
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessor, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr f_cloud = pointProcessor->FilterCloud(inputCloud, 0.25, Eigen::Vector4f (-10, -5, -2, 1), Eigen::Vector4f (25, 7, 2, 1));

  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmented_cloud = pointProcessor->SegmentPlane(f_cloud, 50, 0.15);

  renderPointCloud(viewer, segmented_cloud.second, "plane", Color(0, 1, 0));

  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters = pointProcessor->Clustering(segmented_cloud.first, 0.45, 10, 500);

  int clusterID = 0;
  std::vector<Color> colors = {Color(1, 0, 0), Color(1, 1, 0), Color(0, 0, 1)};

  for(auto cluster : clusters)
  {
    renderPointCloud(viewer, cluster, "obstacle"+clusterID, colors[clusterID%colors.size()]);
    Box clusterBox = pointProcessor->BoundingBox(cluster);
    renderBox(viewer, clusterBox, clusterID);
    clusterID++;
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
  {
    viewer->addCoordinateSystem (1.0);
  }
}


int main (int argc, char** argv)
{
  std::cout << "starting enviroment" << std::endl;

  // Logic for this function was obtained from the Udacity course
  
  ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);

  while (!viewer->wasStopped ())
  {
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();

    inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
    cityBlock(viewer, pointProcessorI, inputCloudI);

    streamIterator++;
    if(streamIterator == stream.end())
    {
      streamIterator = stream.begin();
    }

    viewer->spinOnce();
  } 
}