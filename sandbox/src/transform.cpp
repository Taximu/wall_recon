#include <iostream>
#include "transform.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#ifndef TRANSFORM
#define TRANSFORM

pcl::ModelCoefficients::Ptr
Transform::getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold)
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
  return coefficients;
}

std::vector<double>
Transform::getVectorW(pcl::ModelCoefficients::Ptr coeff)
{
  std::vector<double> K;
  K.push_back(coeff->values[0]);
  K.push_back(coeff->values[1]);
  K.push_back(coeff->values[2]);
  return normalizeVector(K);
}

std::vector<double>
Transform::getVectorV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, pcl::ModelCoefficients::Ptr coeff)
{
  pcl::PointXYZRGB point1, point2;
  std::vector<double> J;

  point1 = findLinePlaneIntersection(coeff, plane->points[0]);
  point2 = findLinePlaneIntersection(coeff, plane->points[1]);
  J.push_back(point2.x - point1.x);
  J.push_back(point2.y - point1.y);
  J.push_back(point2.z - point1.z);
  return normalizeVector(J);
}

pcl::PointXYZRGB
Transform::findLinePlaneIntersection(pcl::ModelCoefficients::Ptr coeffs, pcl::PointXYZRGB directVec)
{
  pcl::PointXYZRGB point;
  double paramT = 0.0;

  paramT = -((coeffs->values[3]) / (coeffs->values[0] * directVec.x + coeffs->values[1] * directVec.y + coeffs->values[2] * directVec.z));
  point.x = directVec.x * paramT;
  point.y = directVec.y * paramT;
  point.z = directVec.z * paramT;
  point.rgb = directVec.rgb;
  return point;
}

std::vector<double>
Transform::getVectorU(std::vector<double> J, std::vector<double> K)
{
  /*
   Find vector I
    |I0 I1 I2|
    |J0 J1 J2|
    |K0 K1 K2|
  */
  std::vector<double> I;
  I.push_back(J.at(1) * K.at(2) - J.at(2) * K.at(1)); // I0
  I.push_back(J.at(2) * K.at(0) - J.at(0) * K.at(2)); // I1
  I.push_back(J.at(0) * K.at(1) - J.at(1) * K.at(0)); // I2
  return normalizeVector(I);
}

std::vector<double>
Transform::normalizeVector(std::vector<double> vec)
{
  std::vector<double> v;
  double vecLength = sqrt(pow(vec.at(0), 2) + pow(vec.at(1), 2) + pow(vec.at(2), 2));
  v.push_back(vec.at(0) / vecLength);
  v.push_back(vec.at(1) / vecLength);
  v.push_back(vec.at(2) / vecLength);
  return v;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Transform::transit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double transitValue)
{
  for (unsigned int i = 0; i < cloud->points.size(); i++)
    cloud->points[i].z += transitValue;
  return cloud;
}

double **
Transform::packVectorsInArray(std::vector<double> I, std::vector<double> J, std::vector<double> K)
{
  double **array = new double *[3];
  array[0] = new double[3];
  array[1] = new double[3];
  array[2] = new double[3];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      if (i == 0)
        array[j][i] = I.at(j);
      else if (i == 1)
        array[j][i] = J.at(j);
      else
        array[j][i] = K.at(j);
    }
  }
  return array;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Transform::rotate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double **arrVec)
{
  pcl::PointXYZRGB point;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    point.x = cloud->points[i].x * arrVec[0][0] + cloud->points[i].y * arrVec[1][0] + cloud->points[i].z * arrVec[2][0];
    point.y = cloud->points[i].x * arrVec[0][1] + cloud->points[i].y * arrVec[1][1] + cloud->points[i].z * arrVec[2][1];
    point.z = cloud->points[i].x * arrVec[0][2] + cloud->points[i].y * arrVec[1][2] + cloud->points[i].z * arrVec[2][2];
    point.rgb = cloud->points[i].rgb;
    cloudPtr->points.push_back(point);
  }
  cloudPtr->width = (int)cloud->points.size();
  cloudPtr->height = 1;
  return cloudPtr;
}
#endif
