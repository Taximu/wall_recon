#include <string>
#include <iostream>
#include "tinyxml.h"
#include <boost/filesystem/convenience.hpp>
#include "feasible_areas_configuration.hpp"

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/feasible_areas.xml";
	this->filePath = str2;
}

int Configuration::dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent)
{
	if (!pParent)
		return 0;

	TiXmlElement *pElement = pParent->ToElement();
	TiXmlAttribute *pAttrib = pElement->FirstAttribute();
	int i = 0;
	int index = 0;
	while (pAttrib)
	{
		if (pParent->ValueStr() == "FirstFile")
		{
			this->inputFile1 = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "SecondFile")
		{
			this->inputFile2 = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "ColorCentroids")
		{
			this->colorCentroids = (pAttrib->ValueStr() == "true") ? 1 : 0;
		}
		else if (pParent->ValueStr() == "ColorAllPointsInCircle")
		{
			this->colorCircle = (pAttrib->ValueStr() == "true") ? 1 : 0;
		}
		else if (pParent->ValueStr() == "BorderColor")
		{
			this->borderColor[index] = pAttrib->IntValue();
			index++;
		}
		else if (pParent->ValueStr() == "FeasibleAreaColor")
		{
			this->feasibleAreaColor[index] = pAttrib->IntValue();
			index++;
		}
		else if (pParent->ValueStr() == "ManipulatorRadius")
		{
			this->manipulatorRadius = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "MiddleSuctionRadius")
		{
			this->middleSuctionRadius = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "OutputFile")
		{
			this->outputFile = pAttrib->Value();
		}
		i++;
		pAttrib = pAttrib->Next();
	}
	return i;
}

void Configuration::dumpToStdout(TiXmlNode *pParent, unsigned int indent)
{
	if (!pParent)
		return;

	TiXmlNode *pChild;
	int t = pParent->Type();

	switch (t)
	{
	case TiXmlNode::TINYXML_DOCUMENT:
		break;
	case TiXmlNode::TINYXML_ELEMENT:
		Configuration::dumpAttribsToStdout(pParent, indent + 1);
		break;
	case TiXmlNode::TINYXML_COMMENT:
		break;
	case TiXmlNode::TINYXML_UNKNOWN:
		break;
	case TiXmlNode::TINYXML_TEXT:
		break;
	case TiXmlNode::TINYXML_DECLARATION:
		break;
	default:
		break;
	}

	for (pChild = pParent->FirstChild(); pChild != 0; pChild = pChild->NextSibling())
		Configuration::dumpToStdout(pChild, indent + 1);
}

void Configuration::dumpToStdout(std::string pFilename)
{
	TiXmlDocument doc(pFilename.c_str());
	if (doc.LoadFile())
		Configuration::dumpToStdout(&doc);
	else
		std::cout << "Failed to load file: " << pFilename << std::endl;
}

void Configuration::outputParameters()
{
	std::cout << "\nConfiguration file [" << this->filePath << "] has the following parameters:\n"
			  << std::endl;
	std::cout.precision(1);
	std::cout << "FirstFile = " << this->inputFile1 << "\n"
			  << "SecondFile = " << this->inputFile2 << "\n"
			  << "ColorCentroids = " << this->colorCentroids << "\n"
			  << "ColorAllPointsInCircle = " << this->colorCircle << "\n"
			  << "BorderColor = (" << this->borderColor[0] << "," << this->borderColor[1] << "," << this->borderColor[2] << ")\n"
			  << "FeasibleAreaColor = (" << this->feasibleAreaColor[0] << "," << this->feasibleAreaColor[1] << "," << this->feasibleAreaColor[2] << ")\n"
			  << "ManipulatorRadius = " << std::fixed << this->manipulatorRadius << "\n"
			  << "MiddleSuctionRadius = " << this->middleSuctionRadius << "\n"
			  << "OutputFile = " << this->outputFile << "\n"
			  << std::endl;
}
#endif
