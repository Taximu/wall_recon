#include <ctime>
#include <string>
#include <iostream>
#include "file_scan.hpp"
#include "transform.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "opencv2/opencv.hpp"
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "orthogonal_projection_configuration.hpp"

void colorScans(const std::string &scansPath);

void project(Configuration &config);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
transit(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointXYZRGB topLeftPt);

bool reside(pcl::PointXYZRGB point, pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt);

double
getDistance(pcl::PointXYZRGB point, pcl::ModelCoefficients::Ptr coeff);

pcl::ModelCoefficients::Ptr
getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold);

pcl::PointXYZRGB
findLinePlaneIntersection(pcl::ModelCoefficients::Ptr coeff, pcl::PointXYZRGB directVec);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

using namespace boost::filesystem;
using namespace cv;

int main(int argc, char *argv[])
{
	clock_t begin = clock();

	Configuration config;

	std::string str = "Preparing for orthogonal_projection...";
	std::cout << std::endl
			  << std::string(str.size(), '#') << std::endl;
	std::cout << str << std::endl;
	std::cout << std::string(str.size(), '#') << std::endl;

	config.setConfigFile();
	config.dumpToStdout(config.filePath);
	config.addParameters(config.filePath);
	config.dumpToStdout(config.filePath);
	config.outputParameters(); // FOR DEBUG PURPOSES
	// colorScans(config.inputPathToScans);
	project(config);
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
	std::cout << "time = " << elapsed_secs << std::endl;
	return 0;
}

void colorScans(const std::string &scansPath)
{
	pcl::PCDReader reader;
	Scanner fileWallsScanner;
	std::string path = scansPath;
	fileWallsScanner.search(path);
	pcl::PointCloud<pcl::PointXYZ>::Ptr loadedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZRGB point_blue;
	uint8_t r(0), g(255), b(255);
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point_blue.rgb = *reinterpret_cast<float *>(&rgb);

	std::list<std::string>::iterator it;
	it = fileWallsScanner.listOfFiles.begin();
	std::string pathFile = "";
	while (it != fileWallsScanner.listOfFiles.end())
	{
		std::cout << "Ready to process!! " << std::flush << std::endl;
		pathFile = path + *it;
		reader.read(pathFile, *loadedCloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		int counter = 0;
		for (unsigned int i = 0; i < loadedCloud->points.size(); i++)
		{
			pcl::PointXYZRGB point;
			point.x = loadedCloud->points[i].x;
			point.y = loadedCloud->points[i].y;
			point.z = loadedCloud->points[i].z;
			point.rgb = point_blue.rgb;
			mainCloud->points.push_back(point);
			counter++;
		}
		mainCloud->width = counter;
		mainCloud->height = 1;
		std::cout << "Writing file: " << pathFile << std::flush << std::endl;
		writeToFile(mainCloud, pathFile);
		it = fileWallsScanner.listOfFiles.erase(it);
	}
}

void project(Configuration &config)
{
	Scanner fileScanner;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mainCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr projectingCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointXYZRGB point_red;
	uint8_t r(255), g(0), b(0);
	uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
	point_red.rgb = *reinterpret_cast<float *>(&rgb);
	pcl::PCDReader reader;

	std::string yamlAlign = config.yamlAlign + "aligned.yaml";
	FileStorage fs(yamlAlign, FileStorage::READ);

	Mat cameraMatrix2;
	fs["transformationMatrix"] >> cameraMatrix2;
	int transitionValue = (int)fs["transitionValue Z"];
	fs.release();

	double **array = new double *[3];
	array[0] = new double[3];
	array[0][0] = cameraMatrix2.at<double>(0, 0);
	array[0][1] = cameraMatrix2.at<double>(0, 1);
	array[0][2] = cameraMatrix2.at<double>(0, 2);
	array[1] = new double[3];
	array[1][0] = cameraMatrix2.at<double>(1, 0);
	array[1][1] = cameraMatrix2.at<double>(1, 1);
	array[1][2] = cameraMatrix2.at<double>(1, 2);
	array[2] = new double[3];
	array[2][0] = cameraMatrix2.at<double>(2, 0);
	array[2][1] = cameraMatrix2.at<double>(2, 1);
	array[2][2] = cameraMatrix2.at<double>(2, 2);

	Scanner fileWallsScanner;
	std::string path = config.pathToWalls;
	fileWallsScanner.search(path);

	std::list<std::string>::iterator it;
	it = fileWallsScanner.listOfFiles.begin();
	while (it != fileWallsScanner.listOfFiles.end())
	{
		std::cout << "\nWorking on wall " << path << *it << std::flush << std::endl;
		std::string pathWall = path + *it;
		reader.read(pathWall, *mainCloud);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

		std::cout << "Cloud size before projection = " << mainCloud->points.size() << std::flush << std::endl;

		Scanner dirScanner;
		std::string path2 = config.inputPathToScans;
		dirScanner.search(path2);
		std::string pathFolder = "";
		std::list<std::string>::iterator iter;
		iter = dirScanner.listOfFiles.begin();
		while (iter != dirScanner.listOfFiles.end())
		{
			pathFolder = path2 + *iter;
			std::cout << "Projecting scan: " << pathFolder << std::flush << std::endl;
			reader.read(pathFolder, *projectingCloud);
			Transform transform;
			projectingCloud = transform.rotate(projectingCloud, array);
			projectingCloud = transform.transit(projectingCloud, transitionValue);

			std::string str(*it);
			std::string str2 = str.substr(0, str.length() - 3);
			str2 += "yaml";
			std::string pathToYamlImage = config.yamlImage + str2;
			FileStorage fs3(pathToYamlImage, FileStorage::READ);

			Mat matrix, transitionValues;
			fs3["transformationMatrix"] >> matrix;
			fs3["transitionValues"] >> transitionValues;
			Mat cameraMatrix;
			transpose(matrix, cameraMatrix);
			fs3.release();

			double **array2 = new double *[3];
			array2[0] = new double[3];
			array2[0][0] = cameraMatrix.at<double>(0, 0);
			array2[0][1] = cameraMatrix.at<double>(0, 1);
			array2[0][2] = cameraMatrix.at<double>(0, 2);
			array2[1] = new double[3];
			array2[1][0] = cameraMatrix.at<double>(1, 0);
			array2[1][1] = cameraMatrix.at<double>(1, 1);
			array2[1][2] = cameraMatrix.at<double>(1, 2);
			array2[2] = new double[3];
			array2[2][0] = cameraMatrix.at<double>(2, 0);
			array2[2][1] = cameraMatrix.at<double>(2, 1);
			array2[2][2] = cameraMatrix.at<double>(2, 2);
			projectingCloud = transform.rotate(projectingCloud, array2);
			pcl::PointXYZRGB topLeftPoint;
			topLeftPoint.x = transitionValues.at<double>(0);
			topLeftPoint.y = transitionValues.at<double>(1);
			topLeftPoint.z = transitionValues.at<double>(2);
			projectingCloud = transit(projectingCloud, topLeftPoint);

			pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
			coeff = getPlaneCoefficients(mainCloud, 5.0f);
			pcl::PointXYZRGB minPt, maxPt;
			pcl::getMinMax3D(*mainCloud, minPt, maxPt);

			int counter = 0;
			for (unsigned int i = 0; i < projectingCloud->points.size(); i++)
			{
				if (reside(projectingCloud->points[i], minPt, maxPt))
				{
					double distance = getDistance(projectingCloud->points[i], coeff);
					if (distance > config.minPointDistance && distance < config.maxPointDistance)
					{
						pcl::PointXYZRGB point;
						point = findLinePlaneIntersection(coeff, projectingCloud->points[i]);
						point.rgb = point_red.rgb;
						point_cloud_ptr->points.push_back(point);
						counter++;
					}
				}
			}
			if (counter > 0)
			{
				point_cloud_ptr->width = counter;
				point_cloud_ptr->height = 1;
				std::cout << "ProjectedPointCloudSize = " << point_cloud_ptr->points.size() << std::flush << std::endl;
				*mainCloud += *point_cloud_ptr;
			}
			iter = fileScanner.listOfFiles.erase(iter);
		}

		std::cout << "Cloud size after projection = " << mainCloud->points.size() << std::flush << std::endl;
		std::cout << "Writing file: " << pathWall << std::flush << std::endl;
		writeToFile(mainCloud, pathWall);
		it = fileWallsScanner.listOfFiles.erase(it);
	}
}

pcl::ModelCoefficients::Ptr
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
	return coefficients;
}

bool reside(pcl::PointXYZRGB point, pcl::PointXYZRGB minPt, pcl::PointXYZRGB maxPt)
{
	return ((point.x >= minPt.x) && (point.x <= maxPt.x) && (point.y >= minPt.y) && (point.y <= maxPt.y) && (point.z >= minPt.z) && (point.z <= maxPt.z));
}

double
getDistance(pcl::PointXYZRGB point, pcl::ModelCoefficients::Ptr coeff)
{
	return fabs(coeff->values[0] * point.x + coeff->values[1] * point.y + coeff->values[2] * point.z + coeff->values[3]) /
		   sqrt(pow(coeff->values[0], 2.0) + pow(coeff->values[1], 2.0) + pow(coeff->values[2], 2.0));
}

pcl::PointXYZRGB
findLinePlaneIntersection(pcl::ModelCoefficients::Ptr coeff, pcl::PointXYZRGB directVec)
{
	pcl::PointXYZRGB point;
	double paramT = 0.0;

	paramT = -1 * (coeff->values[0] * directVec.x + coeff->values[1] * directVec.y + coeff->values[2] * directVec.z + coeff->values[3]) // -1 was in front
			 /
			 (pow(coeff->values[0], 2.0) + pow(coeff->values[1], 2.0) + pow(coeff->values[2], 2.0));
	point.x = directVec.x + (coeff->values[0] * paramT);
	point.y = directVec.y + (coeff->values[1] * paramT);
	point.z = directVec.z + (coeff->values[2] * paramT);
	return point;
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
