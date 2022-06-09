#include <sstream>
#include <iostream>
#include "file_scan.hpp"
#include "boost/progress.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/filesystem/operations.hpp"

#define BOOST_FILESYSTEM_NO_DEPRECATED

#ifndef SCANNER
#define SCANNER

template <typename T>
std::string
Scanner::toString(const T &x)
{
  std::ostringstream os;
  os << x;
  return os.str();
}

void Scanner::search(std::string pathToFolder)
{
  namespace fs = boost::filesystem;
  std::string file;
  fs::path full_path(fs::initial_path<fs::path>());
  full_path = fs::system_complete(fs::path(pathToFolder.c_str()));

  if (!fs::exists(full_path))
  {
    std::cout << "\nNot found: " << full_path.string() << std::endl;
    return;
  }

  if (fs::is_directory(full_path))
  {
    fs::directory_iterator end_iter;
    for (fs::directory_iterator dir_itr(full_path); dir_itr != end_iter; ++dir_itr)
    {
      try
      {
        if (fs::is_regular_file(dir_itr->status()))
        {
          file = toString(dir_itr->path().filename());
          file = file.substr(0, file.length() - 1); // delete symbol " at the beginning
          file = file.substr(1, file.length());     // delete symbol " at the end
          this->listOfFiles.push_back(file);
        }
        else if (fs::is_directory(dir_itr->status()))
        {
          file = toString(dir_itr->path().filename());
          file = file.substr(0, file.length() - 1);
          file = file.substr(1, file.length());
          this->listOfFiles.push_back(file);
        }
      }
      catch (const std::exception &ex)
      {
        std::cout << dir_itr->path().filename() << " " << ex.what() << std::endl;
      }
    }
  }
  else
    std::cout << "\nFound: " << full_path.string() << "\n";
  this->listOfFiles.sort();
}

void Scanner::outputFileNames(std::string pathToFolder)
{
  std::cout << "\nIn directory [" << pathToFolder << "] following files:" << std::endl;
  std::list<std::string>::iterator it;
  for (it = this->listOfFiles.begin(); it != this->listOfFiles.end(); ++it)
  {
    std::cout << ' ' << *it;
    std::cout << '\n';
  }
  std::cout << '\n'
            << std::endl;
}
#endif
