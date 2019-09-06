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

// Ransac helper function definition
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();

  // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
  typename pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT>);

  pcl::VoxelGrid<PointT> vg;
  vg.setInputCloud(cloud);
  vg.setLeafSize(filterRes, filterRes, filterRes);
  vg.filter(*filtered_cloud);

  typename pcl::PointCloud<PointT>::Ptr cropped_cloud (new pcl::PointCloud<PointT>);
  pcl::CropBox<PointT> cropper(true);
  cropper.setMin(minPoint);
  cropper.setMax(maxPoint);
  cropper.setInputCloud(filtered_cloud);
  cropper.filter(*cropped_cloud);

  std::vector<int> indices;

  pcl::CropBox<PointT> roof(true);
  roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
  roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
  roof.setInputCloud(cropped_cloud);
  roof.filter(indices);

  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  for(int point : indices)
  {
    inliers->indices.push_back(point); 
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cropped_cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cropped_cloud);

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

  return cropped_cloud;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
  typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
  typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());

  for(auto c : inliers->indices)
  {
      planeCloud->points.push_back(cloud->points[c]);
  }

  pcl::ExtractIndices<PointT> extract;
  extract.setInputCloud(cloud);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*obstCloud);

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
  return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
  // Time segmentation process
  auto startTime = std::chrono::steady_clock::now();
  pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
  // TODO:: Fill in this function to find inliers for the cloud.

  std::unordered_set<int> ransacResult = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
  inliers->indices.insert(inliers->indices.end(), ransacResult.begin(), ransacResult.end());

  if(inliers->indices.size() == 0) std::cout << "Could not estimate a planar model for the dataset." << std::endl;

  auto endTime = std::chrono::steady_clock::now();
  auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
  return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

  // Time clustering process
  auto startTime = std::chrono::steady_clock::now();

  std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
  std::vector<pcl::PointIndices> clusterIndices;
  std::vector<std::vector<int>> clusterResults;
  std::vector<std::vector<float>> pointsAsVector;
  KdTree* tree = new KdTree;

  // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
  for(int i = 0; i < cloud->points.size(); i++)
  {
    PointT point = cloud->points[i];
    std::vector<float> vectorPoint = {point.x, point.y, point.z};
    pointsAsVector.push_back(vectorPoint);
    tree->insert(vectorPoint, i);
  }

  clusterResults = euclideanCluster(pointsAsVector, tree, clusterTolerance);

  for(auto indexSet : clusterResults)
  {
    if(indexSet.size() < minSize || indexSet.size() > maxSize)
    {
        continue; 
    }

    typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
    
    for(auto index : indexSet)
    {
      cluster->points.push_back(cloud->points[index]);
    }
  
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;
  
    clusters.push_back(cluster);
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