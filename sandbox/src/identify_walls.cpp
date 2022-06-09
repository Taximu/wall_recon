#include <cmath>
#include <ctime>
#include <iostream>
#include "file_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <pcl/ModelCoefficients.h>
#include "identify_walls_configuration.hpp"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>

#define PI 3.14159265

void createDirectories(Configuration &config);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName);

void identifyWalls(Scanner &scanner, Configuration &config);

std::string
createDirectory(Configuration &config, int number);

std::vector<float>
getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold, int maxIterations);

bool sameSign(const double &x, const double &y);

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName);

void locateWallFolders(Configuration &config);

bool copyDir(boost::filesystem::path const &source, boost::filesystem::path const &destination);

int main()
{
    clock_t begin = clock();
    Configuration config;
    Scanner scanner;

    std::string str = "Preparing for identifying walls...";
    std::cout << std::endl
              << std::string(str.size(), '#') << std::endl;
    std::cout << str << std::endl;
    std::cout << std::string(str.size(), '#') << std::endl;

    config.setConfigFile();
    config.dumpToStdout(config.filePath);
    config.addParameters(config.filePath, config.inputPathForWalls);
    config.dumpToStdout(config.filePath);
    // config.outputParameters(); //FOR DEBUG PURPOSES
    scanner.search(config.inputPathForWalls);
    // scanner.outputFileNames(config.inputPathForWalls); //FOR DEBUG PURPOSES
    createDirectories(config);
    identifyWalls(scanner, config);
    locateWallFolders(config);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Running time of identifying: " << elapsed_secs << std::endl;
    return 0;
}

void identifyWalls(Scanner &scanner, Configuration &config)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr compareCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    int numOfFolders = 1;
    std::string outputPath;
    std::vector<float> coeff;
    std::vector<float> coeff2;

    std::list<std::string>::iterator it;
    std::list<std::string>::iterator iter;
    while (it != scanner.listOfFiles.end())
    {
        std::cout << "\nIdentifying wall...\n";
        it = scanner.listOfFiles.begin();
        cloudPtr = loadFile(config.inputPathForWalls + *it);
        coeff = getPlaneCoefficients(cloudPtr, config.distanceThreshold, config.maxIterations);
        outputPath = createDirectory(config, numOfFolders);
        writeToFile(cloudPtr, outputPath + *it);
        it = scanner.listOfFiles.erase(it);

        iter = scanner.listOfFiles.begin();
        while (iter != scanner.listOfFiles.end())
        {
            std::cout << "*";
            std::cout << std::flush;
            std::flush(std::cout);
            compareCloudPtr = loadFile(config.inputPathForWalls + *iter);
            coeff2 = getPlaneCoefficients(compareCloudPtr, config.distanceThreshold, config.maxIterations);
            double numerator = coeff[0] * coeff2[0] + coeff[1] * coeff2[1] + coeff[2] * coeff2[2];
            double denominator = sqrt(pow(coeff[0], 2.0) + pow(coeff[1], 2.0) + pow(coeff[2], 2.0)) *
                                 sqrt(pow(coeff2[0], 2.0) + pow(coeff2[1], 2.0) + pow(coeff2[2], 2.0));
            double cosine = numerator / denominator;
            double angle = acos(cosine) * 180.0 / PI;

            if (sameSign(coeff[3], coeff2[3]) && (angle < config.angle) && (fabs(fabs(coeff[3]) - fabs(coeff2[3])) < 20.0))
            {
                writeToFile(compareCloudPtr, outputPath + *iter);
                iter = scanner.listOfFiles.erase(iter);
            }
            else if (!sameSign(coeff[3], coeff2[3]))
            {
                coeff2[0] *= -1;
                coeff2[1] *= -1;
                coeff2[2] *= -1;
                coeff2[3] *= -1;

                double numerator1 = coeff[0] * coeff2[0] + coeff[1] * coeff2[1] + coeff[2] * coeff2[2];
                double denominator1 = sqrt(pow(coeff[0], 2.0) + pow(coeff[1], 2.0) + pow(coeff[2], 2.0)) *
                                      sqrt(pow(coeff2[0], 2.0) + pow(coeff2[1], 2.0) + pow(coeff2[2], 2.0));
                double cosine2 = numerator1 / denominator1;
                double angle2 = acos(cosine2) * 180.0 / PI;
                if (angle2 < config.angle && (fabs(fabs(coeff[3]) - fabs(coeff2[3])) < 20.0))
                {
                    writeToFile(compareCloudPtr, outputPath + *iter);
                    iter = scanner.listOfFiles.erase(iter);
                }
                else
                {
                    iter++;
                }
            }
            else
            {
                iter++;
            }
        }
        numOfFolders++;
    }
}

void createDirectories(Configuration &config)
{
    boost::filesystem::create_directories(config.outputPathForWalls);
    boost::filesystem::create_directories(config.outputPathForFourWalls);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
loadFile(std::string fileName)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDReader reader;
    reader.read(fileName, *cloudPtr);
    return cloudPtr;
}

std::string
createDirectory(Configuration &config, int number)
{
    std::stringstream sstm;
    sstm << config.outputPathForWalls << "wall" << number << "/";
    std::string folderPath = sstm.str();
    boost::filesystem::create_directories(folderPath);
    return folderPath;
}

std::vector<float>
getPlaneCoefficients(pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane, double distanceThreshold, int maxIterations)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(plane);
    seg.segment(*inliers, *coefficients);
    return coefficients->values;
}

bool sameSign(const double &x, const double &y)
{
    return (x >= 0) ^ (y < 0);
}

void writeToFile(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr, std::string fileName)
{
    pcl::PCDWriter writer;
    writer.write(fileName, *cloudPtr, false);
}

void locateWallFolders(Configuration &config)
{
    Scanner folderScan;
    folderScan.search(config.outputPathForWalls);
    std::list<std::string>::iterator iter;

    std::cout << "\nLocating wall_folders. Wait some minutes..." << std::endl;
    int folderNumber = 1;
    std::list<std::string>::iterator it;
    for (it = folderScan.listOfFiles.begin(); it != folderScan.listOfFiles.end(); ++it)
    {
        std::string folderPath = config.outputPathForWalls + *it + "/";
        Scanner fileScan;
        fileScan.search(folderPath);

        std::list<std::string>::iterator iter = fileScan.listOfFiles.begin();
        std::string filePath = folderPath + *iter;
        if (fileScan.listOfFiles.size() >= 2 || boost::filesystem::file_size(filePath) > 10485760) // more than 10 MBytes
        {
            std::string number = boost::lexical_cast<std::string>(folderNumber);
            std::string outputPath = config.outputPathForFourWalls + "wall" + number + "/";
            copyDir(folderPath.c_str(), outputPath.c_str());
            folderNumber++;
        }
    }
}

bool copyDir(boost::filesystem::path const &source, boost::filesystem::path const &destination)
{
    namespace fs = boost::filesystem;
    try
    {
        if (!fs::exists(source) || !fs::is_directory(source))
        {
            std::cerr << "Source directory " << source.string() << " does not exist or is not a directory." << '\n';
            return false;
        }
        if (fs::exists(destination))
        {
            std::cerr << "Destination directory " << destination.string() << " already exists." << '\n';
            return false;
        }
        if (!fs::create_directory(destination))
        {
            std::cerr << "Unable to create destination directory" << destination.string() << '\n';
            return false;
        }
    }
    catch (fs::filesystem_error const &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
    for (fs::directory_iterator file(source); file != fs::directory_iterator(); ++file)
    {
        try
        {
            fs::path current(file->path());
            if (fs::is_directory(current))
            {
                if (!copyDir(current, destination / current.filename()))
                {
                    return false;
                }
            }
            else
            {
                fs::copy_file(current, destination / current.filename());
            }
        }
        catch (fs::filesystem_error const &e)
        {
            std::cerr << e.what() << '\n';
        }
    }
    return true;
}
