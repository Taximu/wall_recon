#include <string>
#include <iostream>
#include "tinyxml.h"
#include "identify_walls_configuration.hpp"
#include <boost/filesystem/convenience.hpp>

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/identify_walls.xml";
	this->filePath = str2;
}

void Configuration::addParameters(const std::string &pFilename, std::string inputPathForScans)
{
	std::string xmlPath = this->filePath;
	std::string temp(xmlPath);
	std::string s = temp.substr(0, temp.length() - 18);
	s += "remove_inclination.xml";
	TiXmlDocument xmlFile(s);
	if (!xmlFile.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle(&xmlFile);
	TiXmlElement *elem = docHandle.FirstChild("remove_inclination").FirstChild("OutputPath").ToElement();
	TiXmlAttribute *pAttrib = elem->FirstAttribute();
	std::string path(pAttrib->Value());
	std::string p = path + "wall_planes/";

	TiXmlDocument doc(pFilename.c_str());
	if (!doc.LoadFile())
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
	TiXmlHandle handle(&doc);
	TiXmlElement *child = handle.FirstChild("identify_walls").FirstChild("InputPathToWalls").ToElement();
	child->SetAttribute("name", p);
	doc.SaveFile(pFilename);

	std::string outputPath;
	child = handle.FirstChild("identify_walls").FirstChild("OutputPathForSeparateWalls").ToElement();
	std::string str2(p);
	outputPath = "";
	outputPath = str2.substr(0, str2.length() - 20);
	outputPath += "identified_walls/possible_walls/";
	child->SetAttribute("name", outputPath);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("identify_walls").FirstChild("OutputPathForFourWalls").ToElement();
	std::string str3(p);
	outputPath = "";
	outputPath = str3.substr(0, str3.length() - 20);
	outputPath += "identified_walls/real_walls/";
	child->SetAttribute("name", outputPath);
	doc.SaveFile(pFilename);
}

int Configuration::dumpAttribsToStdout(TiXmlNode *pParent, unsigned int indent)
{
	if (!pParent)
		return 0;

	TiXmlElement *pElement = pParent->ToElement();
	TiXmlAttribute *pAttrib = pElement->FirstAttribute();
	int i = 0;
	while (pAttrib)
	{
		if (pParent->ValueStr() == "InputPathToWalls")
		{
			this->inputPathForWalls = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "MaxIterations")
		{
			this->maxIterations = pAttrib->IntValue();
		}
		else if (pParent->ValueStr() == "DistanceThreshold")
		{
			this->distanceThreshold = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "AllowedAngleBetweenPlanes")
		{
			this->angle = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "OutputPathForSeparateWalls")
		{
			this->outputPathForWalls = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "OutputPathForFourWalls")
		{
			this->outputPathForFourWalls = pAttrib->Value();
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
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
}

void Configuration::outputParameters()
{
	std::cout << "\nConfiguration file [" << this->filePath << "] has the following parameters:\n"
			  << std::endl;
	std::cout.precision(1);
	std::cout << "InputPathToWalls = " << this->inputPathForWalls << "\n"
			  << "MaxIterations = " << this->maxIterations << "\n"
			  << "DistanceThreshold = " << std::fixed << this->distanceThreshold << "\n"
			  << "AllowedAngleBetweenPlanes = " << this->angle << "\n"
			  << "OutputPathForSeparateWalls = " << this->outputPathForWalls << "\n"
			  << "OutputPathForFourWalls = " << this->outputPathForFourWalls << std::endl;
}
#endif
