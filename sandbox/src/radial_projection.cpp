#include <math.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include "radial_projection_configuration.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

std::vector<float>
getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
projectPointsOnPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Configuration &config);

float getDistancePointToPlane(pcl::PointXYZRGB point, std::vector<float> coeffs);

bool resideOnPlane(pcl::PointXYZRGB point, std::vector<float> coeffs);

pcl::PointXYZRGB
findLinePlaneIntersection(std::vector<float> planeCoeffs, pcl::PointXYZRGB directVec);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

int main(int argc, char **argv)
{
  Configuration config;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

  config.setConfigFile();
  config.dumpToStdout(config.filePath);
  config.outputParameters();

  cloudPtr = loadFile(config.inputFile);
  cloudPtr = projectPointsOnPlane(cloudPtr, config);
  writeToFile(cloudPtr, config.outputFile);
  return 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PCDReader reader;
  reader.read(fileName, *cloudPtr);
  return cloudPtr;
}

std::vector<float>
getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold)
{
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;

  seg.setOptimizeCoefficients(false);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(distanceThreshold);
  seg.setInputCloud(plane);
  seg.segment(*inliers, *coefficients);
  return coefficients->values;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
projectPointsOnPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Configuration &config)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
  int cloudWidthCounter = 0;
  pcl::PointXYZRGB point;
  std::vector<float> coeffs = getPlaneCoefficients(cloud, config.distanceThreshold);

  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    if (resideOnPlane(cloud->points[i], coeffs))
    {
      finalCloudPtr->points.push_back(cloud->points[i]);
      cloudWidthCounter++;
      continue;
    }
    else
    {
      point = findLinePlaneIntersection(coeffs, cloud->points[i]);
      finalCloudPtr->points.push_back(point);
      cloudWidthCounter++;
    }
  }
  finalCloudPtr->width = cloudWidthCounter;
  finalCloudPtr->height = 1;
  return finalCloudPtr;
}

bool resideOnPlane(pcl::PointXYZRGB point, std::vector<float> coeffs)
{
  return ((coeffs[0] * point.x + coeffs[1] * point.y + coeffs[2] * point.z + coeffs[3]) == 0) ? 1 : 0;
}

float getDistancePointToPlane(pcl::PointXYZRGB point, std::vector<float> coeffs)
{
  float numerator = abs(coeffs[0] * point.x + coeffs[1] * point.y + coeffs[2] * point.z + coeffs[3]);
  float denominator = sqrt(pow(coeffs[0], 2.0) + pow(coeffs[1], 2.0) + pow(coeffs[2], 2.0));
  return numerator / denominator;
}

pcl::PointXYZRGB
findLinePlaneIntersection(std::vector<float> coeffs, pcl::PointXYZRGB directVec)
{
  pcl::PointXYZRGB point;
  float paramT = 0.0;

  paramT = -((coeffs[3]) / (coeffs[0] * directVec.x + coeffs[1] * directVec.y + coeffs[2] * directVec.z));
  point.x = directVec.x * paramT;
  point.y = directVec.y * paramT;
  point.z = directVec.z * paramT;
  point.rgb = directVec.rgb;
  return point;
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB>(fileName, *cloudPtr, false);
}
