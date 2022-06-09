#ifndef VOXEL_GRID_CONFIGURATION_HPP
#define VOXEL_GRID_CONFIGURATION_HPP

#include "tinyxml.h"
#include <string>

class Configuration
{
public:
	std::string filePath;
	std::string inputFile;
	std::string outputFile;
	int borderColor[3];
	double leafSize;

	void setConfigFile();
	int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
	void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
	void dumpToStdout(std::string pFilename);
	void outputParameters();
};
#endif
