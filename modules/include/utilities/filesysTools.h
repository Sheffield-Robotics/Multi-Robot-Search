/***************************************************************************
 *            filesysTools.h
 *
 *  Fr 02 Mär 2007 23:14:21 CET
 *  Copyright 2007 Rainer Kümmerle
 *  Email rk@raikue.net
 ****************************************************************************/

#ifndef FILESYS_TOOLS_H
#define FILESYS_TOOLS_H

/** @addtogroup util libutil **/
// @{

/** \file filesysTools.h
 * \brief utility functions for handling files, directory on Linux/Unix
 */

#include <string>

/**
 * get filename extension (the part after the last .), e.g.
 * the extension of file.txt is txt
 */
std::string getFileExtension(const std::string& filename);

/**
 * get the filename without the extension.
 * file.txt -> file
 */
std::string getPureFilename(const std::string& filename);

/**
 * change the fileextension of a given filename.
 * Only if filename contains an extension, otherwise filename is returned.
 */
std::string changeFileExtension(const std::string& filename, const std::string& newExt);

/**
 * return the basename of the given filename
 * /etc/fstab -> fstab
 */
std::string getBasename(const std::string& filename);

/**
 * return the directory of a given filename
 * /etc/fstab -> /etc
 */
std::string getDirname(const std::string& filename);

/**
 * check if file exists (note a directory is also a file)
 */
bool fileExists(const char* filename);
bool fileExists(const std::string & filename);

/**
 * checks if file exists and is a file
 */
bool isRegularFile(const char* filename);
bool isRegularFile(const std::string & filename);

/**
 * is the given filename a valid direcory, e.g. exists
 */
bool isDirectory(const char* filename);
bool isDirectory(const std::string & filename);

/**
 * return the current date as a filename of the form YYYYMMDD_hhmmss
 */
std::string getCurrentDateAsFilename();

/**
 * \return the next free filename in a series, e.g. if data000, data002, data003 exist in the current directory, getNextFreeEnumeratedFilename("data") returns data001.
 */
std::string getNextFreeEnumeratedFilename(std::string prefix);

/** return the modification date of a file */
time_t getLastModificationDate(const char* filename);

/** return date of last access to a file */
time_t getLastAccessDate(const char* filename);

/** return date of last status change of a file */
time_t getLastStatusChangeDate(const char* filename);

/**
 * create a directory. if pub is true, then the rights of the dir
 * will be rwxrwxrwx (readable and writable for everyone)
 */
bool createDirectory(const char* dirName, bool pub= false);

// @}
#endif
