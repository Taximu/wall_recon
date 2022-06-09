#ifndef FEASIBLE_AREAS_CONFIGURATION_HPP
#define FEASIBLE_AREAS_CONFIGURATION_HPP

#include <string>
#include "tinyxml.h"

class Configuration
{
public:
  std::string filePath;
  std::string inputFile1;
  std::string inputFile2;
  std::string outputFile;
  int borderColor[3];
  int feasibleAreaColor[3];
  double manipulatorRadius;
  double middleSuctionRadius;
  bool colorCentroids;
  bool colorCircle;

  void setConfigFile();
  int dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent);
  void dumpToStdout(TiXmlNode *pParent, unsigned int indent = 0);
  void dumpToStdout(std::string fileName);
  void addParameters(const std::string &pFilename, std::string pathForScans);
  void outputParameters();
};
#endif
