#include <iostream>
#include <tinyxml.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "feasible_areas_configuration.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
concatenate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
findFeasibleAreas(pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloudPtr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr edgesPtr, Configuration &config);

bool findRedPointInCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, pcl::PointXYZRGB center);

void colorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, pcl::PointXYZRGB center);

double
getDistance(pcl::PointXYZRGB point, pcl::PointXYZRGB center);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

pcl::PointXYZRGB pink_point;

int main(int argc, char *argv[])
{
	Configuration config;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr edges_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

	config.setConfigFile();
	config.dumpToStdout(config.filePath);
	config.outputParameters();

	cloud_ptr = loadFile(config.inputFile1);
	edges_ptr = loadFile(config.inputFile2);
	final_cloud_ptr = concatenate(cloud_ptr, edges_ptr);
	final_cloud_ptr = findFeasibleAreas(final_cloud_ptr, edges_ptr, config);
	writeToFile(final_cloud_ptr, config.outputFile);
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
concatenate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_resulting(new pcl::PointCloud<pcl::PointXYZRGB>);
	*cloud_resulting += *cloud1;
	*cloud_resulting += *cloud2;
	return cloud_resulting;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
findFeasibleAreas(pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud_ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr edges_ptr, Configuration &config)
{
	pcl::PointXYZRGB red_point;
	uint8_t r = config.borderColor[0], g = config.borderColor[1], b = config.borderColor[2];
	uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
	red_point.rgb = *reinterpret_cast<float *>(&rgb);

	uint8_t r1 = config.feasibleAreaColor[0], g1 = config.feasibleAreaColor[1], b1 = config.feasibleAreaColor[2];
	uint32_t rgb1 = ((uint32_t)r1 << 16 | (uint32_t)g1 << 8 | (uint32_t)b1);
	pink_point.rgb = *reinterpret_cast<float *>(&rgb1);

	double radius = (config.manipulatorRadius > 0) ? config.manipulatorRadius : config.middleSuctionRadius;
	for (unsigned int i = 0; i < final_cloud_ptr->points.size(); i++)
	{
		if (final_cloud_ptr->points[i].rgb != red_point.rgb)
		{
			if (findRedPointInCircle(edges_ptr, radius, final_cloud_ptr->points[i]) == false)
			{
				if (config.colorCentroids)
				{
					if (final_cloud_ptr->points[i].rgb != pink_point.rgb)
						final_cloud_ptr->points[i].rgb = pink_point.rgb;
				}
				else
				{
					colorPoints(final_cloud_ptr, radius, final_cloud_ptr->points[i]);
				}
				continue;
			}
		}
	}
	return final_cloud_ptr;
}

bool findRedPointInCircle(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, pcl::PointXYZRGB center)
{
   for (unsigned int i = 0; i < cloud->points.size(); 
   {
		if (getDistance(cloud->points[i], center) <= pow(radius, 2.0))
			return true;
   }
   return false;
}

void colorPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double radius, pcl::PointXYZRGB center)
{
	for (unsigned int i = 0; i < cloud->points.size(); i++)
	{
		if (cloud->points[i].rgb != pink_point.rgb && getDistance(cloud->points[i], center) <= pow(radius, 2.0))
			cloud->points[i].rgb = pink_point.rgb;
	}
}

double
getDistance(pcl::PointXYZRGB point, pcl::PointXYZRGB center)
{
	return pow((point.x - center.x), 2.0) + pow((point.y - center.y), 2.0) + pow((point.z - center.z), 2.0);
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
	pcl::PCDWriter writer;
	writer.write(fileName, *cloudPtr, false);
}
