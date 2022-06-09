#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "voxel_grid_configuration.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
discretize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, Configuration &config);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

int main(int argc, char **argv)
{
   Configuration config;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

   config.setConfigFile();
   config.dumpToStdout(config.filePath);
   // config.outputParameters(); //FOR DEBUG PURPOSES
   cloud = loadFile(config.inputFile);
   cloud_filtered = discretize(cloud, config);
   writeToFile(cloud_filtered, config.outputFile);
   return (0);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PCDReader reader;
   reader.read(fileName, *cloudPtr);
   return cloudPtr;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
discretize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Configuration &config)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_red(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_blue(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB red_point;
   uint8_t r = config.borderColor[0], g = config.borderColor[1], b = config.borderColor[2];
   uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
   red_point.rgb = *reinterpret_cast<float *>(&rgb);

   pcl::PointXYZRGB point;
   int redPointCounter = 0, bluePointCounter = 0;
   for (unsigned int i = 0; i < cloud->points.size(); i++)
   {
      if (cloud->points[i].rgb == red_point.rgb)
      {
         point.x = cloud->points[i].x;
         point.y = cloud->points[i].y;
         point.z = cloud->points[i].z;
         point.rgb = cloud->points[i].rgb;
         cloud_red->points.push_back(point);
         redPointCounter++;
      }
      else
      {
         point.x = cloud->points[i].x;
         point.y = cloud->points[i].y;
         point.z = cloud->points[i].z;
         point.rgb = cloud->points[i].rgb;
         cloud_blue->points.push_back(point);
         bluePointCounter++;
      }
   }

   cloud_red->width = redPointCounter;
   cloud_red->height = 1;
   cloud_blue->width = bluePointCounter;
   cloud_blue->height = 1;

   pcl::VoxelGrid<pcl::PointXYZRGB> sor;
   sor.setInputCloud(cloud_blue);
   sor.setLeafSize(config.leafSize, config.leafSize, config.leafSize);
   sor.filter(*cloud);
   *cloud += *cloud_red;
   return cloud;
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZRGB>(fileName, *cloudPtr, false);
}
