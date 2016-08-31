#pragma once
#include <cstdlib>
#include <stdio.h>
#include <string>
#include <fstream>

/*!
A static class for common errors related to opening files.
*/

class FileErrors{
public:
	//! Error that prints and pauses when a variable is not found.
	static void variable_not_found(std::string var_name){
		std::printf("Variable %s not found!\n",var_name.c_str());
		std::system("pause");
	}

	//! Error that prints when a file was not opened.
	static void failed_file(std::string file_name){
		std::printf("Failed to open %s.\n",file_name.c_str());
	}

	//! Error that prints, pauses, and closes the program when an extension is not recognized.
	static void unrecognized_extension(std::string extension_name){
		std::printf("Unknown extension %s: please specify separator type. Aborting.\n",extension_name.c_str());
		std::system("pause");
	}

	//! Error that prints, pauses, and closes the program when pair detection fails.
	static void not_pair(std::string file_name){
		std::printf("Error! %s does not contain pair values. Aborting.\n",file_name.c_str());
		std::system("pause");
	}

private:
	//! Error codes for the different fatal failures.
	enum ErrorCodes{
		UNRECOGNIZED_EXTENSION,
		NOT_PAIR
	};
};
