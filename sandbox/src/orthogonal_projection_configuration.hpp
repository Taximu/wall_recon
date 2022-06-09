#ifndef ORTHOGONAL_PROJECTION_CONFIGURATION_HPP
#define ORHTOGONAL_PROJECTION_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
  std::string filePath;
  std::string inputPathToScans;
  std::string pathToWalls;
  std::string outputPath;
  std::string yamlAlign;
  std::string yamlImage;
  double minPointDistance;
  double maxPointDistance;

  void setConfigFile();
  int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
  void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
  void dumpToStdout(std::string pFilename);
  void addParameters(const std::string &pFilename);
  void outputParameters();
};
#endif
