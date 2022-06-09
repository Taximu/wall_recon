#ifndef CREATE_IMAGE_CONFIGURATION_HPP
#define CREATE_IMAGE_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
	std::string filePath;
	std::string inputPath;
	std::string outputPathForImage;
	std::string outputPathForInfoFile;
	int borderColor[3];
	int feasibleAreaColor[3];
	int emptyAreaColor[3];
	int wallColor[3];
	double pixelSize;

	void setConfigFile();
	int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
	void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
	void dumpToStdout(std::string pFilename);
	void addParameters(const std::string &pFilename);
	void outputParameters();
};
#endif
