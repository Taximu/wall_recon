#include <string>
#include <iostream>
#include "tinyxml.h"
#include <boost/filesystem/convenience.hpp>
#include "orthogonal_projection_configuration.hpp"

#ifndef CONFIGURATION
#define CONFIGURATION

void Configuration::setConfigFile()
{
	boost::filesystem::path runPath;
	runPath = boost::filesystem::initial_path();
	std::string str(runPath.string());
	std::string str2 = str.substr(0, str.length() - 3);
	str2 += "config/orthogonal_projection.xml";
	this->filePath = str2;
}

void Configuration::addParameters(const std::string &pFilename)
{
	std::string xmlPath = this->filePath;
	std::string temp(xmlPath);
	std::string s = temp.substr(0, temp.length() - 25);
	s += "sac.xml";
	TiXmlDocument xmlFile(s);
	if (!xmlFile.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle(&xmlFile);
	TiXmlElement *elem = docHandle.FirstChild("SAC").FirstChild("InputPathForScans").ToElement();
	TiXmlAttribute *pAttrib = elem->FirstAttribute();
	std::string path(pAttrib->Value());
	std::string pathToScansFolder = path;

	std::string temp2(xmlPath);
	s = "";
	s = temp2.substr(0, temp2.length() - 25);
	s += "remove_inclination.xml";
	TiXmlDocument xmlFile2(s);
	if (!xmlFile2.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;

	TiXmlHandle docHandle2(&xmlFile2);
	TiXmlElement *elem2 = docHandle2.FirstChild("remove_inclination").FirstChild("PathForYamlAligned").ToElement();
	TiXmlAttribute *pAttrib2 = elem2->FirstAttribute();
	std::string path2(pAttrib2->Value());
	std::string pathToYamlFile = path2;

	std::string temp3(xmlPath);
	s = "";
	s = temp3.substr(0, temp3.length() - 25);
	s += "preparation_for_image_creation.xml";
	TiXmlDocument xmlFile3(s);
	if (!xmlFile3.LoadFile())
		std::cout << "\nFailed to load file = " << s << std::endl;
	TiXmlHandle docHandle3(&xmlFile3);
	TiXmlElement *elem3 = docHandle3.FirstChild("preparation_for_image_creation").FirstChild("OutputPathWalls").ToElement();
	TiXmlAttribute *pAttrib3 = elem3->FirstAttribute();
	std::string path3(pAttrib3->Value());
	std::string pathToWalls = path3;

	elem3 = docHandle3.FirstChild("preparation_for_image_creation").FirstChild("OutputPathInfoFile").ToElement();
	pAttrib3 = elem3->FirstAttribute();
	std::string path4(pAttrib3->Value());
	std::string pathToYamlImage = path4;

	TiXmlDocument doc(pFilename.c_str());
	if (!doc.LoadFile())
		std::cout << "\nFailed to load file = " << pFilename << std::endl;
	TiXmlHandle handle(&doc);
	TiXmlElement *child = handle.FirstChild("orthogonal_projection").FirstChild("InputPathToScans").ToElement();
	child->SetAttribute("name", pathToScansFolder);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("orthogonal_projection").FirstChild("PathToYamlAligned").ToElement();
	child->SetAttribute("name", pathToYamlFile);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("orthogonal_projection").FirstChild("PathToYamlImage").ToElement();
	child->SetAttribute("name", pathToYamlImage);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("orthogonal_projection").FirstChild("PathToWalls").ToElement();
	child->SetAttribute("name", pathToWalls);
	doc.SaveFile(pFilename);

	child = handle.FirstChild("orthogonal_projection").FirstChild("OutputPath").ToElement();
	child->SetAttribute("name", pathToWalls);
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
		if (pParent->ValueStr() == "InputPathToScans")
		{
			this->inputPathToScans = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "PathToYamlAligned")
		{
			this->yamlAlign = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "PathToYamlImage")
		{
			this->yamlImage = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "PathToWalls")
		{
			this->pathToWalls = pAttrib->Value();
		}
		else if (pParent->ValueStr() == "MinDistanceToPlane")
		{
			this->minPointDistance = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "MaxDistanceToPlane")
		{
			this->maxPointDistance = pAttrib->DoubleValue();
		}
		else if (pParent->ValueStr() == "OutputPath")
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
	std::cout << "InputPathToScans = " << this->inputPathToScans << "\n"
			  << "PathToYamlAligned = " << this->yamlAlign << "\n"
			  << "PathToYamlImage = " << this->yamlImage << "\n"
			  << "PathToWalls = " << this->pathToWalls << "\n"
			  << "MinPointDistance = " << this->minPointDistance << "\n"
			  << "MaxPointDistance = " << this->maxPointDistance << "\n"
			  << "OutputPath = " << this->outputPath << "\n"
			  << std::endl;
}
#endif
