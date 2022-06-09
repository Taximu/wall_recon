#include <ctime>
#include <math.h>
#include <time.h>
#include <iostream>
#include "transform.hpp"
#include "file_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include "preparation_for_creating_image_configuration.hpp"

#define PI 3.14159265

void createDirectories(Configuration &config);

void preparation(Scanner &fileScanner, Configuration &config, std::string floorPath);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
transit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB topLeftPt);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

using namespace cv;

int main()
{
  clock_t begin = clock();
  Configuration config;
  Scanner fileScanner;
  Scanner floorFileScanner;

  std::string str = "Preparing for transforming walls for image creation...";
  std::cout << std::endl
            << std::string(str.size(), '#') << std::endl;
  std::cout << str << std::endl;
  std::cout << std::string(str.size(), '#') << std::endl;

  config.setConfigFile();
  config.dumpToStdout(config.filePath);
  // config.addParameters(config.filePath);
  // config.dumpToStdout(config.filePath);
  // config.outputParameters(); //FOR DEBUG PURPOSES
  fileScanner.search(config.inputPathForWalls);
  // dirScanner.outputFileNames(config.inputPathForWalls); //FOR DEBUG PURPOSES
  floorFileScanner.search(config.inputPathForFloorFile);
  std::list<std::string>::iterator it;
  it = floorFileScanner.listOfFiles.begin();
  std::string floorPath = config.inputPathForFloorFile + *it;
  createDirectories(config);
  preparation(fileScanner, config, floorPath);
  clock_t end = clock();
  double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
  std::cout << "Running time: " << elapsed_secs << std::endl;
  return 0;
}

void createDirectories(Configuration &config)
{
  boost::filesystem::create_directories(config.outputPathForWalls);
  boost::filesystem::create_directories(config.outputPathForInfoFile);
}

void preparation(Scanner &fileScanner, Configuration &config, std::string floorPath)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFloor(new pcl::PointCloud<pcl::PointXYZRGB>);
  Transform transform;

  cloudFloor = loadFile(floorPath);
  pcl::ModelCoefficients::Ptr coeff2 = transform.getPlaneCoefficients(cloudFloor, config.distanceThreshold);
  std::vector<double> K = transform.getVectorW(coeff2);

  int fileNumber = 1;
  std::list<std::string>::iterator it;
  it = fileScanner.listOfFiles.begin();
  while (it != fileScanner.listOfFiles.end())
  {
    std::string filePath = config.inputPathForWalls + *it;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloudPtr = loadFile(filePath);
    std::string message = "Transforming: " + filePath;
    std::cout << std::endl
              << std::string(message.size(), '=') << std::endl;
    std::cout << message << std::endl;
    std::cout << std::string(message.size(), '=') << std::endl
              << std::flush << std::endl;

    pcl::ModelCoefficients::Ptr coeff = transform.getPlaneCoefficients(cloudPtr, config.distanceThreshold);
    std::vector<double> J = transform.getVectorW(coeff);
    std::vector<double> I = transform.getVectorU(J, K);
    K = transform.getVectorU(I, J);
    double **ptrArr = transform.packVectorsInArray(I, J, K);

    std::stringstream ss;
    ss << config.outputPathForInfoFile << "wall" << fileNumber << ".yaml";
    std::string infoFilePath = ss.str();
    ss.str("");
    FileStorage fs(infoFilePath, FileStorage::WRITE);
    time_t rawtime;
    time(&rawtime);
    fs << "Date" << asctime(localtime(&rawtime));
    Mat transformMatrix = (Mat_<double>(3, 3) << ptrArr[0][0], ptrArr[0][1], ptrArr[0][2],
                           ptrArr[1][0], ptrArr[1][1], ptrArr[1][2],
                           ptrArr[2][0], ptrArr[2][1], ptrArr[2][2]);
    Mat transposeMatrix;
    transpose(transformMatrix, transposeMatrix);
    fs << "transformationMatrix" << transposeMatrix;
    cloudPtr = transform.rotate(cloudPtr, ptrArr);
    pcl::PointXYZRGB minPt, maxPt;
    pcl::getMinMax3D(*cloudPtr, minPt, maxPt);
    pcl::PointXYZRGB topLeftPoint;
    topLeftPoint.x = minPt.x;
    topLeftPoint.y = minPt.y;
    topLeftPoint.z = minPt.z;
    topLeftPoint.x *= -1;
    topLeftPoint.y *= -1;
    topLeftPoint.z *= -1;
    Mat distCoeffs = (Mat_<double>(3, 1) << topLeftPoint.x, topLeftPoint.y, topLeftPoint.z);
    fs << "transitionValues" << distCoeffs;
    fs.release();

    cloudPtr = transit(cloudPtr, topLeftPoint);
    ss << config.outputPathForWalls << "wall" << fileNumber << ".pcd";
    std::string outputWallPath = ss.str();
    writeToFile(cloudPtr, outputWallPath);

    it = fileScanner.listOfFiles.erase(it);
    delete[] ptrArr;
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
transit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB topLeftPt)
{
  for (unsigned int i = 0; i < cloud->points.size(); i++)
  {
    cloud->points[i].x += topLeftPt.x;
    cloud->points[i].y += topLeftPt.y;
    cloud->points[i].z += topLeftPt.z;
  }
  return cloud;
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGB>(fileName, *cloudPtr, false);
}
