#pragma once
#include <list>

/*!
A static class that modifies filepaths.
*/

class FilePath{
public:
	//! Removes the filepath to the file
	static void strip_file_path(std::string &file_name){
		std::istringstream iss(file_name);
		std::string token;
		while (std::getline(iss,token,'/')); // Returns the last element in the filepath
		file_name = token;
	}

	//! Strips the extension from the filepath.
	static void strip_extension(std::string &file_name){
		size_t found;
		found = file_name.find('.');
		file_name.resize(found);
	}
};
