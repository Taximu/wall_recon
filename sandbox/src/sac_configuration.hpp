#ifndef SAC_CONFIGURATION_HPP
#define SAC_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

class Configuration
{
public:
	std::string filePath;
	std::string inputPathForScans;
	std::string outputPathForWalls;
	std::string outputPathForFloor;
	std::string outputPathForRoof;
	int planeColor[3];
	int maxIterarions;
	int numberOfPlanesToExtract;
	int modelType;	// store same values as in pcl/sample_consensus/model_types.h
	int methodType; // store same values as in pcl/sample_consensus/method_types.h
	double distanceThreshold;

	void setConfigFile();
	int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
	void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
	void dumpToStdout(std::string pFilename);
	void addParameters(const std::string &pFilename, std::string pathForScans);
	void outputParameters();
};
#endif
