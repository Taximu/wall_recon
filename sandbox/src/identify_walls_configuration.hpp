#ifndef IDENTIFY_WALLS_CONFIGURATION_HPP
#define IDENTIFY_WALLS_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
  std::string filePath;
  std::string inputPathForWalls;
  std::string outputPathForWalls;
  std::string outputPathForFourWalls;
  double distanceThreshold;
  double angle;
  int maxIterations;

  void setConfigFile();
  int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
  void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
  void dumpToStdout(std::string pFilename);
  void addParameters(const std::string &pFilename, std::string inputPathForScans);
  void outputParameters();
};
#endif
