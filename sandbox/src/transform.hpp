#ifndef TRASNFORM_HPP
#define TRASNFORM_HPP

#include <string>
#include "tinyxml.h"
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

class Transform
{
public:
  pcl::ModelCoefficients::Ptr
  getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold);

  std::vector<double>
  getVectorW(pcl::ModelCoefficients::Ptr coeff);

  std::vector<double>
  getVectorV(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, pcl::ModelCoefficients::Ptr coeff);

  pcl::PointXYZRGB
  findLinePlaneIntersection(pcl::ModelCoefficients::Ptr coeffs, pcl::PointXYZRGB directVec);

  std::vector<double>
  getVectorU(std::vector<double> J, std::vector<double> K);

  std::vector<double>
  normalizeVector(std::vector<double> vec);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  transit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double transitValue);

  double **
  packVectorsInArray(std::vector<double> I, std::vector<double> J, std::vector<double> K);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr
  rotate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double **arrVec);
};
#endif
