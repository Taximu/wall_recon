#include <string>
#include <iostream>
#include "tinyxml.h"
#include <boost/filesystem/convenience.hpp>
#include "preparation_for_creating_image_configuration.hpp"

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/preparation_for_image_creation.xml";
	this->filePath = str2;
}

void Configuration::addParameters(const std::string &pFilename)
{
	std::string xmlPath = this->filePath;
	std::string temp(xmlPath);
	std::string s = temp.substr(0, temp.length() - 34);
	s += "concatenate.xml";
	TiXmlDocument xmlFile(s);
	if (!xmlFile.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle(&xmlFile);
	TiXmlElement *elem = docHandle.FirstChild("concatenate").FirstChild("OutputFile").ToElement();
	TiXmlAttribute *pAttrib = elem->FirstAttribute();
	std::string path(pAttrib->Value());
	std::string p = path;

	std::string temp2(xmlPath);
	s = "";
	s = temp2.substr(0, temp2.length() - 34);
	s += "remove_inclination.xml";
	TiXmlDocument xmlFile2(s);
	if (!xmlFile2.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle2(&xmlFile2);
	TiXmlElement *elem2 = docHandle2.FirstChild("remove_inclination").FirstChild("OutputPath").ToElement();
	TiXmlAttribute *pAttrib2 = elem2->FirstAttribute();
	std::string path2(pAttrib2->Value());
	std::string p2 = path2 + "floor/";

	TiXmlDocument doc(pFilename.c_str());
	if (!doc.LoadFile())
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
	TiXmlHandle handle(&doc);
	TiXmlElement *child = handle.FirstChild("preparation_for_image_creation").FirstChild("InputPathWalls").ToElement();
	child->SetAttribute("name", p);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("preparation_for_image_creation").FirstChild("InputFloorFile").ToElement();
	std::string str2(p2);
	child->SetAttribute("name", p2);
	doc.SaveFile(pFilename);

	std::string outputPath;
	child = handle.FirstChild("preparation_for_image_creation").FirstChild("OutputPathWalls").ToElement();
	std::string str3(p);
	outputPath = p + "transformed/walls/";
	child->SetAttribute("name", outputPath);
	doc.SaveFile(pFilename);

	outputPath = "";
	child = handle.FirstChild("preparation_for_image_creation").FirstChild("OutputPathInfoFile").ToElement();
	std::string str4(p);
	outputPath = p + "transformed/wall_images/";
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
		if (pParent->ValueStr() == "InputPathWalls")
		{
			this->inputPathForWalls = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "InputFloorFile")
		{
			this->inputPathForFloorFile = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "DistanceThreshold")
		{
			this->distanceThreshold = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "OutputPathWalls")
		{
			this->outputPathForWalls = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "OutputPathInfoFile")
		{
			this->outputPathForInfoFile = pAttrib->Value();
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
	std::cout << "InputPathWalls = " << this->inputPathForWalls << "\n"
			  << "InputFloorFile = " << this->inputPathForFloorFile << "\n"
			  << "DistanceThreshold = " << std::fixed << this->distanceThreshold << "\n"
			  << "OutputPathForWalls = " << this->outputPathForWalls << "\n"
			  << "OutputPathForInfoFile = " << this->outputPathForInfoFile << std::endl;
}
#endif
