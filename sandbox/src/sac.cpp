#include <ctime>
#include <string>
#include <iostream>
#include "file_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include "sac_configuration.hpp"
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr
loadFile(std::string fileName);

void createDirectories(Configuration &config);

void extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr, Configuration &config, int &fileNumber);

void colorCloudAndOutput(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int &fileNumber, int pos, Configuration &config);

int main(int argc, char **argv)
{
   clock_t begin = clock();
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob(new pcl::PointCloud<pcl::PointXYZ>);
   Configuration config;
   Scanner scanner;
   int fileNumber = 0;

   std::string str = "Preparing for segmentation...";
   std::cout << std::endl
             << std::string(str.size(), '#') << std::endl;
   std::cout << str << std::endl;
   std::cout << std::string(str.size(), '#') << std::endl;

   config.setConfigFile();
   config.dumpToStdout(config.filePath);
   if (argc >= 2)
   {
      config.addParameters(config.filePath, argv[1]);
   }
   else
   {
      config.addParameters(config.filePath, config.inputPathForScans);
   }
   config.dumpToStdout(config.filePath);
   // config.outputParameters(); //FOR DEBUG PURPOSES
   scanner.search(config.inputPathForScans);
   // scanner.outputFileNames(config.inputPathForScans); //FOR DEBUG PURPOSES
   createDirectories(config);

   std::list<std::string>::iterator it;
   for (it = scanner.listOfFiles.begin(); it != scanner.listOfFiles.end(); ++it)
   {
      cloud_blob = loadFile(config.inputPathForScans + *it);
      std::string strLoadedFile = "Loaded file: " + config.inputPathForScans + *it;
      std::cout << std::endl
                << std::string(strLoadedFile.size(), '=') << std::endl;
      std::cout << strLoadedFile << std::endl;
      std::cout << std::string(strLoadedFile.size(), '=') << std::endl
                << std::endl;
      extractPlanes(cloud_blob, config, fileNumber);

      /////////////////////// for KOED
      clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      std::cout << "Running time: " << elapsed_secs << std::endl;
      return 0;
   }
   clock_t end = clock();
   double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   std::cout << "Running time: " << elapsed_secs << std::endl;
   return 0;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
loadFile(std::string fileName)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PCDReader reader;
   reader.read(fileName, *cloudPtr);
   return cloudPtr;
}

void createDirectories(Configuration &config)
{
   boost::filesystem::create_directories(config.outputPathForFloor);
   boost::filesystem::create_directories(config.outputPathForRoof);
   boost::filesystem::create_directories(config.outputPathForWalls);
}

void extractPlanes(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_blob, Configuration &config, int &fileNumber)
{
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
   pcl::SACSegmentation<pcl::PointXYZ> seg;
   int pos[3] = {001, 010, 100}; // floor, wall, roof
   pcl::PointXYZ minPt, maxPt;

   seg.setOptimizeCoefficients(false);
   seg.setModelType(config.modelType);
   seg.setMethodType(config.methodType);
   seg.setMaxIterations(config.maxIterarions);
   seg.setDistanceThreshold(config.distanceThreshold);

   pcl::ExtractIndices<pcl::PointXYZ> extract;
   int num = 0;
   while (num <= config.numberOfPlanesToExtract)
   {
      std::cout << "Extracting planes..." << std::endl;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud<pcl::PointXYZ>(*cloud_blob, *cloud);
      seg.setInputCloud(cloud);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() == 0)
      {
         std::cerr << "Could not estimate a planar model for the given dataset anymore." << std::endl;
         break;
      }

      // Extract the inliers
      extract.setInputCloud(cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*cloud_p);
      pcl::getMinMax3D(*cloud_p, minPt, maxPt);
      if ((fabs(coefficients->values[2]) > 0.8) && (minPt.z <= 0))
      {
         colorCloudAndOutput(cloud_p, fileNumber, pos[0], config);
      }
      else if ((fabs(coefficients->values[2]) > 0.8) && (minPt.z > 0))
      {
         colorCloudAndOutput(cloud_p, fileNumber, pos[2], config);
      }
      else
      {
         colorCloudAndOutput(cloud_p, fileNumber, pos[1], config);
      }

      // Create the filtering object
      extract.setNegative(true);
      extract.filter(*cloud_f);
      cloud_blob.swap(cloud_f);
      fileNumber++;
      num++;
   }
}

void colorCloudAndOutput(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int &fileNumber, int pos, Configuration &config)
{
   std::stringstream ss;
   if (pos == 001)
      ss << config.outputPathForFloor << "plane_" << fileNumber << ".pcd";
   else if (pos == 010)
      ss << config.outputPathForWalls << "plane_" << fileNumber << ".pcd";
   else
      ss << config.outputPathForRoof << "plane_" << fileNumber << ".pcd";
   std::string planePath = ss.str();
   std::cout << "Writing plane to a file: " << planePath << std::endl;

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB point;
   uint8_t r(config.planeColor[0]), g(config.planeColor[1]), b(config.planeColor[2]);
   uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
   point.rgb = *reinterpret_cast<float *>(&rgb);
   for (unsigned int i = 0; i < cloud->points.size(); i++)
   {
      point.x = cloud->points[i].x;
      point.y = cloud->points[i].y;
      point.z = cloud->points[i].z;
      point_cloud_ptr->points.push_back(point);
   }
   point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
   point_cloud_ptr->height = 1;

   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZRGB>(planePath, *point_cloud_ptr, false);
}
