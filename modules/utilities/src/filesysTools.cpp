/***************************************************************************
 *            filesysTools.cpp
 *
 *  Fr 02 Mär 2007 23:14:08 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/
#include "utilities/filesysTools.h"
#include <sys/stat.h>
#include <ctime>
#include <vector>
#include <dirent.h>
#include <sstream>
#include <iomanip>
#include <stdio.h>

std::string getFileExtension(const std::string& filename)
{
  std::string::size_type lastDot = filename.find_last_of('.');
  if (lastDot != std::string::npos) 
    return filename.substr(lastDot + 1);
  else
    return "";
}

std::string getPureFilename(const std::string& filename)
{
  std::string::size_type lastDot = filename.find_last_of('.');
  if (lastDot != std::string::npos) 
    return filename.substr(0, lastDot);
  else
    return filename;
}

std::string getBasename(const std::string& filename)
{
  std::string::size_type lastSlash = filename.find_last_of('/');
  if (lastSlash != std::string::npos) 
    return filename.substr(lastSlash + 1);
  else
    return filename;
}

std::string getDirname(const std::string& filename)
{
  std::string::size_type lastSlash = filename.find_last_of('/');
  if (lastSlash != std::string::npos) 
    return filename.substr(0, lastSlash);
  else
    return "";
}

std::string changeFileExtension(const std::string& filename, const std::string& newExt)
{
  std::string::size_type lastDot = filename.find_last_of('.');
  if (lastDot != std::string::npos) 
    return filename.substr(0, lastDot + 1) + newExt;
  else
    return filename;
}

bool fileExists(const char* filename)
{
  struct stat statInfo;
  return (stat(filename, &statInfo) == 0);
}
bool fileExists(const std::string & filename)
{
   return fileExists(filename.c_str());
}

bool isRegularFile(const char* filename)
{
  struct stat statInfo;
  return (stat(filename, &statInfo) == 0 && S_ISREG(statInfo.st_mode));
}
bool isRegularFile(const std::string & filename)
{
   return isRegularFile(filename.c_str());
}

bool isDirectory(const char* filename)
{
  struct stat statInfo;
  return (stat(filename, &statInfo) == 0 && S_ISDIR(statInfo.st_mode));
}
bool isDirectory(const std::string & filename)
{
   return isDirectory(filename.c_str());
}

std::string getCurrentDateAsFilename()
{
  time_t t = time(NULL);
  const size_t dateStrSize = 1024;
  char dateStr[dateStrSize];
  if (strftime(dateStr, dateStrSize, "%Y%m%d_%H%M%S", localtime(&t)) == 0)
    fprintf(stderr, "Error (%s: %s) Date: %s\n", __FILE__, __func__, dateStr);
  return std::string(dateStr);
}

std::string getNextFreeEnumeratedFilename(std::string prefix)
{
   DIR* curDir = opendir(".");
   if(curDir == NULL)
      return "";

   std::vector<std::string> dirEntries;
   struct dirent* dirEnt = readdir(curDir);
   while(dirEnt != NULL) {
      std::string dup = dirEnt->d_name;
      dirEntries.push_back(dup);
      dirEnt = readdir(curDir);
   }

   for(unsigned int i = 0; i < 1000; i++) {
      std::stringstream ss;
      ss << prefix << std::setw(3) << std::setfill('0') << i;
      bool found = false;
      for(unsigned int j = 0; j < dirEntries.size(); j++) {
         std::string cur = dirEntries.at(j);
         if(cur.compare(0, ss.str().length(), ss.str()) == 0) {
            found = true;
            break;
         }
      }
      if(!found)
         return ss.str();
   }
   fprintf(stderr, "Warning (%s: %s) Couldn't determine next free filename for %s\n", __FILE__, __func__, prefix.c_str());
   return prefix;
}

time_t getLastModificationDate(const char* filename)
{
  struct stat statInfo;
  if (stat(filename, &statInfo) == 0) {
    return statInfo.st_mtime;
  } else {
    return 0;
  }
}

time_t getLastAccessDate(const char* filename)
{
  struct stat statInfo;
  if (stat(filename, &statInfo) == 0) {
    return statInfo.st_atime;
  } else {
    return 0;
  }
}

time_t getLastStatusChangeDate(const char* filename)
{
  struct stat statInfo;
  if (stat(filename, &statInfo) == 0) {
    return statInfo.st_ctime;
  } else {
    return 0;
  }
}

bool createDirectory(const char* dirName, bool pub)
{
  bool status = true;
  status &= (mkdir(dirName, 0) == 0);
  if (pub)
    status &= (0 == chmod(dirName, // set directory to rwxrwxrwx
          S_IRUSR | S_IWUSR | S_IXUSR |
          S_IRGRP | S_IWGRP | S_IXGRP |
          S_IROTH | S_IWOTH | S_IXOTH ));
  return status;
}
