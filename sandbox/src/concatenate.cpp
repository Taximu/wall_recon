#include <ctime>
#include <iostream>
#include "file_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <boost/filesystem.hpp>
#include "concatenate_configuration.hpp"

void createDirectory(std::string dirName);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

void concatenatePlanes(Scanner &dirScanner, Configuration &config);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

int main(int argc, char **argv)
{
   clock_t begin = clock();
   Configuration config;
   Scanner dirScanner;

   std::string str = "Preparing for concatenation...";
   std::cout << std::endl
             << std::string(str.size(), '#') << std::endl;
   std::cout << str << std::endl;
   std::cout << std::string(str.size(), '#') << std::endl;

   config.setConfigFile();
   config.dumpToStdout(config.filePath);
   config.addParameters(config.filePath, config.inputPath);
   config.dumpToStdout(config.filePath);
   // config.outputParameters(); //FOR DEBUG PURPOSES
   dirScanner.search(config.inputPath);
   // dirScanner.outputFileNames(config.inputPath); //FOR DEBUG PURPOSES
   createDirectory(config.outputPath);
   concatenatePlanes(dirScanner, config);
   clock_t end = clock();
   double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
   std::cout << "Running time: " << elapsed_secs << std::endl;
   return (0);
}

void createDirectory(std::string dirName)
{
   const char *dir_path = (dirName).c_str();
   boost::filesystem::path dir(dir_path);
   boost::filesystem::create_directory(dir);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName)
{
   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PCDReader reader;
   reader.read(fileName, *cloudPtr);
   return cloudPtr;
}

void concatenatePlanes(Scanner &dirScanner, Configuration &config)
{
   int fileNumber = 1;
   std::list<std::string>::iterator it;
   std::list<std::string>::iterator iter;
   it = dirScanner.listOfFiles.begin();
   while (it != dirScanner.listOfFiles.end())
   {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      std::string folderPath = config.inputPath + *it + "/";
      std::string message = "Working on folder: " + folderPath;
      std::cout << std::endl
                << std::string(message.size(), '=') << std::endl;
      std::cout << message << std::endl;
      std::cout << std::string(message.size(), '=') << std::endl
                << std::endl;

      Scanner fileScanner;
      fileScanner.search(folderPath);
      std::list<std::string>::iterator iter;
      iter = fileScanner.listOfFiles.begin();
      while (iter != fileScanner.listOfFiles.end())
      {
         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
         std::string filePath = config.inputPath + *it + "/" + *iter;
         std::cout << "Concatenating file: " << filePath << std::endl;
         cloudPtr = loadFile(filePath);
         *mainCloud += *cloudPtr;
         iter = fileScanner.listOfFiles.erase(iter);
      }
      std::stringstream ss;
      ss << config.outputPath << "wall" << fileNumber << ".pcd";
      std::string outputFile = ss.str();
      std::cout << "Writting file: " << outputFile << std::endl;
      writeToFile(mainCloud, outputFile);
      fileNumber++;
      it = dirScanner.listOfFiles.erase(it);
   }
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
   pcl::PCDWriter writer;
   writer.write<pcl::PointXYZRGB>(fileName, *cloudPtr, false);
}
