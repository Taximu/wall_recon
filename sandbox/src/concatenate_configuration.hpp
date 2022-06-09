#ifndef CONCATENATE_CONFIGURATION_HPP
#define CONCATENATE_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
	std::string filePath;
	std::string inputPath;
	std::string outputPath;

	void setConfigFile();
	int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
	void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
	void dumpToStdout(std::string pFilename);
	void addParameters(const std::string &pFilename, std::string inputPathForScans);
	void outputParameters();
};
#endif
