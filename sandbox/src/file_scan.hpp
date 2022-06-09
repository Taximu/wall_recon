#ifndef FILE_SCAN_HPP
#define FILE_SCAN_HPP

#include <list>
#include <string>

class Scanner
{
public:
	std::list<std::string> listOfFiles;
	template <typename T>
	std::string toString(const T &x);
	void search(std::string pathToFolder);
	void outputFileNames(std::string pathToFolder);
};
#endif
