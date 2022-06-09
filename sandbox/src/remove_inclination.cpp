#include <ctime>
#include <math.h>
#include <iostream>
#include "transform.hpp"
#include "file_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include "remove_inclination_configuration.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

void createDirectory(std::string dirName);

void removeInclination(Scanner &dirScanner, Configuration &config, std::string floorPath);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

using namespace cv;

int main()
{
   clock_t begin = clock();
   Configuration config;
   Scanner dirScanner;
   Scanner floorFileScanner;

   std::string str = "Preparing for transformation...";
   std::cout << std::endl
             << std::string(str.size(), '#') << std::endl;
   std::cout << str << std::endl;
   std::cout << std::string(str.size(), '#') << std::endl;

   config.setConfigFile();
   config.dumpToStdout(config.filePath);
   config.addParameters(config.filePath, config.inputPathForSAC);
   config.dumpToStdout(config.filePath);
   // config.outputParameters(); //FOR DEBUG PURPOSES
   dirScanner.search(config.inputPathForSAC);
   // dirScanner.outputFileNames(config.inputPathForSAC); //FOR DEBUG PURPOSES
   floorFileScanner.search(config.inputPathForFloorFile);
   std::list<std::string>::iterator it;
   it = floorFileScanner.listOfFiles.begin();
   std::string floorPath = config.inputPathForFloorFile + *it;
   createDirectory(config.outputPathForAligned);
   removeInclination(dirScanner, config, floorPath);
   clock_t end = clock();
   double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   std::cout << "Running time: " << elapsed_secs << std::endl;
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

void createDirectory(std::string dirName)
{
   const char *dir_path = (dirName).c_str();
   boost::filesystem::path dir(dir_path);
   boost::filesystem::create_directory(dir);
}

void removeInclination(Scanner &dirScanner, Configuration &config, std::string floorPath)
{
   Transform transform;
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::ModelCoefficients::Ptr coeff;
   std::vector<double> J, K, I;
   double **ptrArr;
   double floorValue = 0;
   double transitValue = 0.0;
   int counter = 0;

   createDirectory(config.pathForYamlAligned);
   mainCloud = loadFile(floorPath);
   coeff = transform.getPlaneCoefficients(mainCloud, config.distanceThreshold);
   K = transform.getVectorW(coeff);
   J = transform.getVectorV(mainCloud, coeff);
   I = transform.getVectorU(J, K);
   ptrArr = transform.packVectorsInArray(I, J, K);

   std::list<std::string>::iterator it;
   std::list<std::string>::iterator iter;
   it = dirScanner.listOfFiles.begin();
   while (it != dirScanner.listOfFiles.end())
   {
      std::string folderPath = config.inputPathForSAC + *it + "/";
      std::string message = "Working on folder: " + folderPath;
      std::cout << std::endl
                << std::string(message.size(), '=') << std::endl;
      std::cout << message << std::endl;
      std::cout << std::string(message.size(), '=') << std::endl
                << std::endl;
      std::string outputPath = config.outputPathForAligned + *it + "/";
      createDirectory(outputPath);
      Scanner fileScanner;
      fileScanner.search(folderPath);
      iter = fileScanner.listOfFiles.begin();
      while (iter != fileScanner.listOfFiles.end())
      {

         std::string filePath = config.inputPathForSAC + *it + "/" + *iter;
         std::cout << "Processing file: " << filePath << std::endl;
         cloudPtr = loadFile(filePath);
         cloudPtr = transform.rotate(cloudPtr, ptrArr);
         std::string currentPlane(*it);

         if ((currentPlane.compare("floor") == 0) && counter == 0)
         {
            pcl::PointXYZRGB minPt, maxPt;
            pcl::getMinMax3D(*cloudPtr, minPt, maxPt);
            floorValue = minPt.z;
            transitValue = floorValue;
            transitValue *= -1;
            counter++;

            std::string yamlOutputPath = config.pathForYamlAligned + "aligned.yaml";
            FileStorage fs(yamlOutputPath, FileStorage::WRITE);
            Mat transformMatrix = (Mat_<double>(3, 3) << ptrArr[0][0], ptrArr[0][1], ptrArr[0][2],
                                   ptrArr[1][0], ptrArr[1][1], ptrArr[1][2],
                                   ptrArr[2][0], ptrArr[2][1], ptrArr[2][2]);
            fs << "transformationMatrix" << transformMatrix;
            fs << "transitionValue Z" << transitValue;
            fs.release();
         }

         cloudPtr = transform.transit(cloudPtr, transitValue);
         std::string outputFile = outputPath + *iter;
         std::cout << "Writting file: " << outputFile << std::endl
                   << std::endl;
         writeToFile(cloudPtr, outputFile);
         iter = fileScanner.listOfFiles.erase(iter);
      }
      it = dirScanner.listOfFiles.erase(it);
   }
   delete[] ptrArr;
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZRGB>(fileName, *cloudPtr, false);
}
