#include <ctime>
#include <ostream>
#include <iostream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/concave_hull.h>
#include "concave_hull_configuration.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
findBorders(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double alpha);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Configuration &config);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

int main(int argc, char **argv)
{
    clock_t tStart = clock();
    Configuration config;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr borderCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);

    config.setConfigFile();
    config.dumpToStdout(config.filePath);
    config.outputParameters();

    cloudPtr = loadFile(config.inputFile);
    borderCloudPtr = findBorders(cloudPtr, config.alpha);
    borderCloudPtr = colorCloud(borderCloudPtr, config);

    writeToFile(borderCloudPtr, config.outputFile);
    printf("Execution Time: %.2fs\n", (double)(clock() - tStart) / CLOCKS_PER_SEC);
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
findBorders(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, double alpha)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::ConcaveHull<pcl::PointXYZRGB> chull;
    chull.setInputCloud(cloud);
    chull.setAlpha(alpha);
    chull.reconstruct(*cloud_hull);
    return cloud_hull;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
colorCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Configuration &config)
{
    pcl::PointXYZRGB point;
    uint8_t r(config.borderColor[0]), g(config.borderColor[1]), b(config.borderColor[2]);
    uint32_t rgb = (static_cast<uint32_t>(r) << 16 | static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
    point.rgb = *reinterpret_cast<float *>(&rgb);

    for (unsigned int i = 0; i < cloud->points.size(); i++)
        cloud->points[i].rgb = point.rgb;
    return cloud;
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
    pcl::PCDWriter writer;
    writer.write(fileName, *cloudPtr, false);
}
