#include <string>
#include <iostream>
#include "tinyxml.h"
#include "create_image_configuration.hpp"
#include <boost/filesystem/convenience.hpp>

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/create_image.xml";
	this->filePath = str2;
}

void Configuration::addParameters(const std::string &pFilename)
{
	std::string xmlPath = this->filePath;
	std::string temp(xmlPath);
	std::string s = temp.substr(0, temp.length() - 16);
	s += "preparation_for_image_creation.xml";
	TiXmlDocument xmlFile(s);
	if (!xmlFile.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle(&xmlFile);
	TiXmlElement *elem = docHandle.FirstChild("preparation_for_image_creation").FirstChild("OutputPathWalls").ToElement();
	TiXmlAttribute *pAttrib = elem->FirstAttribute();
	std::string path(pAttrib->Value());
	std::string p = path;

	TiXmlElement *elem2 = docHandle.FirstChild("preparation_for_image_creation").FirstChild("OutputPathInfoFile").ToElement();
	TiXmlAttribute *pAttrib2 = elem2->FirstAttribute();
	std::string path2(pAttrib2->Value());
	std::string p2 = path2;

	TiXmlDocument doc(pFilename.c_str());
	if (!doc.LoadFile())
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
	TiXmlHandle handle(&doc);
	TiXmlElement *child = handle.FirstChild("create_image").FirstChild("InputFile").ToElement();
	child->SetAttribute("name", p);
	doc.SaveFile(pFilename);

	std::string outputPath;
	child = handle.FirstChild("create_image").FirstChild("OutputPathForImage").ToElement();
	child->SetAttribute("name", p2);
	doc.SaveFile(pFilename);

	outputPath = "";
	child = handle.FirstChild("create_image").FirstChild("OutputPathForInfoFile").ToElement();
	child->SetAttribute("name", p2);
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
		if (pParent->ValueStr() == "InputFile")
		{
			this->inputPath = pAttrib->Value();
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
		else if (pParent->ValueStr() == "EmptyAreaColor")
		{
			this->emptyAreaColor[index] = pAttrib->IntValue();
			index++;
		}
		else if (pParent->ValueStr() == "WallColor")
		{
			this->wallColor[index] = pAttrib->IntValue();
			index++;
		}
		else if (pParent->ValueStr() == "PixelSize")
		{
			this->pixelSize = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "OutputPathForImage")
		{
			this->outputPathForImage = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "OutputPathForInfoFile")
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
	std::cout << "InputFile = " << this->inputPath << "\n"
			  << "BorderColor = (" << this->borderColor[0] << ","
			  << this->borderColor[1] << ","
			  << this->borderColor[2]
			  << ")\n"
			  << "FeasibleAreaColor = (" << this->feasibleAreaColor[0] << ","
			  << this->feasibleAreaColor[1] << ","
			  << this->feasibleAreaColor[2]
			  << ")\n"
			  << "EmptyAreaColor = (" << this->emptyAreaColor[0] << ","
			  << this->emptyAreaColor[1] << ","
			  << this->emptyAreaColor[2]
			  << ")\n"
			  << "WallColor = (" << this->wallColor[0] << ","
			  << this->wallColor[1] << ","
			  << this->wallColor[2]
			  << ")\n"
			  << "PixelSize = " << std::fixed << this->pixelSize << "\n"
			  << "OutputPathForImage= " << this->outputPathForImage << "\n"
			  << "OutputPathForInfoFile = " << this->outputPathForInfoFile << "\n"
			  << std::endl;
}
#endif
