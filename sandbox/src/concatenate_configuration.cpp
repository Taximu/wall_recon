#include <string>
#include <iostream>
#include "tinyxml.h"
#include "concatenate_configuration.hpp"
#include <boost/filesystem/convenience.hpp>

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/concatenate.xml";
	this->filePath = str2;
}

void Configuration::addParameters(const std::string &pFilename, std::string inputPathForScans)
{
	std::string xmlPath = this->filePath;
	std::string temp(xmlPath);
	std::string s = temp.substr(0, temp.length() - 15);
	s += "identify_walls.xml";
	TiXmlDocument xmlFile(s);
	if (!xmlFile.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle(&xmlFile);
	TiXmlElement *elem = docHandle.FirstChild("identify_walls").FirstChild("OutputPathForFourWalls").ToElement();
	TiXmlAttribute *pAttrib = elem->FirstAttribute();
	std::string path(pAttrib->Value());
	std::string p = path;

	TiXmlDocument doc(pFilename.c_str());
	if (!doc.LoadFile())
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
	TiXmlHandle handle(&doc);
	TiXmlElement *child = handle.FirstChild("concatenate").FirstChild("InputFile").ToElement();
	child->SetAttribute("name", p);
	doc.SaveFile(pFilename);

	std::string outputPath;
	child = handle.FirstChild("concatenate").FirstChild("OutputFile").ToElement();
	std::string str2(p);
	outputPath = "";
	outputPath = str2.substr(0, str2.length() - 28);
	outputPath += "processed_walls/";
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
		if (pParent->ValueStr() == "InputFile")
		{
			this->inputPath = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "OutputFile")
		{
			this->outputPath = pAttrib->Value();
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
	std::cout << "InputPath = " << this->inputPath << "\n"
			  << "OutputPath = " << this->outputPath << "\n"
			  << std::endl;
}
#endif
