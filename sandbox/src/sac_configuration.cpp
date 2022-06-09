#include <string>
#include <iostream>
#include "tinyxml.h"
#include "sac_configuration.hpp"
#include <boost/filesystem/convenience.hpp>

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/sac.xml";
	this->filePath = str2.c_str();
}

void Configuration::addParameters(const std::string &pFilename, std::string inputPathForScans)
{
	TiXmlDocument doc(pFilename.c_str());
	if (!doc.LoadFile())
		std::cout << "\nFailed to load file = " << pFilename << std::endl;

	TiXmlHandle handle(&doc);
	TiXmlElement *child = handle.FirstChild("SAC").FirstChild("InputPathForScans").ToElement();
	child->SetAttribute("name", inputPathForScans);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("SAC").FirstChild("OutputPathForFloor").ToElement();
	std::string str(inputPathForScans);
	std::string outputPath = "";
	std::string key("/");
	unsigned found = str.rfind(key);

	if (found == str.size() - 1)
	{
		str.replace(found, key.length(), "");
	}
	found = str.rfind(key);
	std::string rootPath = str.substr(0, str.length() - (str.length() - found));
	outputPath = rootPath + "/sac/floor/";
	child->SetAttribute("name", outputPath);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("SAC").FirstChild("OutputPathForRoof").ToElement();
	std::string str1(inputPathForScans);
	outputPath = "";
	outputPath = rootPath + "/sac/roof/";
	child->SetAttribute("name", outputPath);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("SAC").FirstChild("OutputPathForWalls").ToElement();
	std::string str2(inputPathForScans);
	outputPath = "";
	outputPath = rootPath + "/sac/wall_planes/";
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
	int index = 0;
	while (pAttrib)
	{
		if (pParent->ValueStr() == "InputPathForScans")
		{
			this->inputPathForScans = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "ModelType")
		{
			if (pAttrib->ValueStr() == "plane")
				this->modelType = 0;
			else if (pAttrib->ValueStr() == "line")
				this->modelType = 1;
			else if (pAttrib->ValueStr() == "circle2D")
				this->modelType = 2;
			else if (pAttrib->ValueStr() == "circle3D")
				this->modelType = 3;
			else if (pAttrib->ValueStr() == "sphere")
				this->modelType = 4;
			else if (pAttrib->ValueStr() == "cylinder")
				this->modelType = 5;
			else if (pAttrib->ValueStr() == "cone")
				this->modelType = 6;
			else if (pAttrib->ValueStr() == "torus")
				this->modelType = 7;
			else if (pAttrib->ValueStr() == "parallel_line")
				this->modelType = 8;
			else if (pAttrib->ValueStr() == "perpen_plane")
				this->modelType = 9;
			else if (pAttrib->ValueStr() == "parallel_lines")
				this->modelType = 10;
			else if (pAttrib->ValueStr() == "normal_plane")
				this->modelType = 11;
			else if (pAttrib->ValueStr() == "registration")
				this->modelType = 12;
			else if (pAttrib->ValueStr() == "parallel_plane")
				this->modelType = 13;
			else if (pAttrib->ValueStr() == "normal_parallel_plane")
				this->modelType = 14;
			else if (pAttrib->ValueStr() == "stick")
				this->modelType = 15;
		}
		else if (pParent->ValueStr() == "MethodType")
		{
			if (pAttrib->ValueStr() == "ransac")
				this->methodType = 0;
			else if (pAttrib->ValueStr() == "mlesac")
				this->methodType = 5;
			else if (pAttrib->ValueStr() == "prosac")
				this->methodType = 6;
		}
		else if (pParent->ValueStr() == "MaxIterations")
		{
			this->maxIterarions = pAttrib->IntValue();
		}
		else if (pParent->ValueStr() == "DistanceThreshold")
		{
			this->distanceThreshold = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "NumberOfPlanesToExtract")
		{
			this->numberOfPlanesToExtract = pAttrib->IntValue();
		}
		else if (pParent->ValueStr() == "PlaneColor")
		{
			this->planeColor[index] = pAttrib->IntValue();
			index++;
		}
		else if (pParent->ValueStr() == "OutputPathForWalls")
		{
			this->outputPathForWalls = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "OutputPathForFloor")
		{
			this->outputPathForFloor = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "OutputPathForRoof")
		{
			this->outputPathForRoof = pAttrib->Value();
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
	bool loadOkay = doc.LoadFile();
	if (loadOkay)
		Configuration::dumpToStdout(&doc);
	else
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
}

void Configuration::outputParameters()
{
	std::cout << "\nConfiguration file [" << this->filePath << "] has the following parameters:\n"
			  << std::endl;
	std::cout.precision(1);
	std::cout << "InputPathForScans = " << this->inputPathForScans << "\n"
			  << "ModelType = " << this->modelType << "\n"
			  << "MethodType = " << this->methodType << "\n"
			  << "MaxIterations = " << this->maxIterarions << "\n"
			  << "DistanceThreshold = " << std::fixed << this->distanceThreshold << "\n"
			  << "numberOfPlanesToExtract = " << this->numberOfPlanesToExtract << "\n"
			  << "PlaneColor = (" << this->planeColor[0] << ","
			  << this->planeColor[1] << ","
			  << this->planeColor[2]
			  << ")\n"
			  << "OutputPathForWalls = " << this->outputPathForWalls << "\n"
			  << "OutputPathForFloor = " << this->outputPathForFloor << "\n"
			  << "OutputPathForRoof = " << this->outputPathForRoof << "\n"
			  << std::endl;
}
#endif
