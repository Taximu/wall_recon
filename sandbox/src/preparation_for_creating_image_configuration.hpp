#ifndef PREPARATION_FOR_CREATING_IMAGE_CONFIGURATION_HPP
#define PREPARATION_FOR_CREATING_IMAGE_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
	std::string filePath;
	std::string inputPathForWalls;
	std::string inputPathForFloorFile;
	std::string outputPathForWalls;
	std::string outputPathForInfoFile;
	double distanceThreshold;

	void setConfigFile();
	int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
	void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
	void dumpToStdout(std::string pFilename);
	void addParameters(const std::string &pFilename);
	void outputParameters();
};
#endif
