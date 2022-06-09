#ifndef RADIAL_PROJECTION_CONFIGURATION_HPP
#define RADIAL_PROJECTION_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
	std::string filePath;
	std::string inputFile;
	std::string outputFile;
	double distanceThreshold;

	void setConfigFile();
	int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
	void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
	void dumpToStdout(std::string pFilename);
	void outputParameters();
};
#endif
