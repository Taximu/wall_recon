#include <ctime>
#include <math.h>
#include <fstream>
#include <iostream>
#include "EasyBMP.h"
#include "file_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include "create_image_configuration.hpp"

void createDirectory(std::string folderName);

void createImage(Configuration &config, Scanner &fileScanner);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

void writeToInfoFile(Configuration &config, int fileNumber);

using namespace cv;

int main(int argc, char **argv)
{
   clock_t begin = clock();
   Configuration config;
   Scanner fileScanner;

   std::string str = "Preparing for creating_images...";
   std::cout << std::endl
             << std::string(str.size(), '#') << std::endl;
   std::cout << str << std::endl;
   std::cout << std::string(str.size(), '#') << std::endl
             << std::endl;

   config.setConfigFile();
   config.dumpToStdout(config.filePath);
   config.addParameters(config.filePath);
   config.dumpToStdout(config.filePath);
   // config.outputParameters(); //FOR DEBUG PURPOSES
   fileScanner.search(config.inputPath);
   // fileScanner.outputFileNames(config.inputPath); //FOR DEBUG PURPOSES
   createDirectory(config.outputPathForImage);
   createImage(config, fileScanner);
   clock_t end = clock();
   double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   std::cout << "Running time: " << elapsed_secs << std::endl;
   return 0;
}

void createDirectory(std::string dirName)
{
   const char *dir_path = (dirName).c_str();
   boost::filesystem::path dir(dir_path);
   boost::filesystem::create_directory(dir);
}

void createImage(Configuration &config, Scanner &fileScanner)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr workCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointXYZRGB red_point;
   uint8_t r = config.borderColor[0], g = config.borderColor[1], b = config.borderColor[2];
   uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
   red_point.rgb = *reinterpret_cast<float *>(&rgb);
   pcl::PointXYZRGB minPt, maxPt;
   int height, width;
   BMP image;

   int fileNumber = 1;
   std::list<std::string>::iterator it;
   it = fileScanner.listOfFiles.begin();
   while (it != fileScanner.listOfFiles.end())
   {
      std::string filePath = config.inputPath + *it;
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr workCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
      workCloudPtr = loadFile(filePath);
      std::cout << "Creating image from: " + filePath + "... " << std::endl
                << std::flush;

      pcl::getMinMax3D(*workCloudPtr, minPt, maxPt);
      width = ceil(maxPt.x / config.pixelSize);
      height = ceil(maxPt.z / config.pixelSize);
      image.SetSize(width, height);
      image.SetBitDepth(32);
      for (int i = 0; i < width; i++)
      {
         for (int j = 0; j < height; j++)
         {
            image(i, j)->Red = config.emptyAreaColor[0];
            image(i, j)->Green = config.emptyAreaColor[1];
            image(i, j)->Blue = config.emptyAreaColor[2];
         }
      }
      for (unsigned int k = 0; k < workCloudPtr->points.size(); k++)
      {
         int i = floor(workCloudPtr->points[k].x / config.pixelSize);
         int j = height - floor(workCloudPtr->points[k].z / config.pixelSize);
         j--;
         if (workCloudPtr->points[k].rgb == red_point.rgb)
         {
            image(i, j)->Red = config.borderColor[0];
            image(i, j)->Green = config.borderColor[1];
            image(i, j)->Blue = config.borderColor[2];
         }
         else if (image(i, j)->Red != config.borderColor[0])
         {
            image(i, j)->Red = config.wallColor[0];
            image(i, j)->Green = config.wallColor[1];
            image(i, j)->Blue = config.wallColor[2];
         }
      }

      std::stringstream ss;
      ss << config.outputPathForImage << "wall" << fileNumber << ".bmp";
      std::string wallFilePath = ss.str();
      image.WriteToFile(wallFilePath.c_str());
      writeToInfoFile(config, fileNumber);
      it = fileScanner.listOfFiles.erase(it);
      fileNumber++;
   }
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PCDReader reader;
   reader.read(fileName, *cloudPtr);
   return cloudPtr;
}

void writeToInfoFile(Configuration &config, int fileNumber)
{
   std::stringstream ss;
   ss << config.outputPathForInfoFile << "wall" << fileNumber << ".yaml";
   std::string infoFilePath = ss.str();

   FileStorage fs(infoFilePath, FileStorage::APPEND);
   fs << "pixelSize" << config.pixelSize;
   fs << "wallColor"
      << "[";
   fs << "{:"
      << "r" << config.wallColor[0] << "g" << config.wallColor[1] << "b" << config.wallColor[2] << "}";
   fs << "]";
   fs << "emptyAreaColor"
      << "[";
   fs << "{:"
      << "r" << config.emptyAreaColor[0] << "g" << config.emptyAreaColor[1] << "b" << config.emptyAreaColor[2] << "}";
   fs << "]";
   fs.release();
}
