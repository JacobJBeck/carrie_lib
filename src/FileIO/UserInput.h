#pragma once
#include <iostream>
#include <string>

/*!
A static class to interact with the user
*/

class UserInput{
public:
	//! Asks 'question' and gets a user input. Takes keyboard entry 'y' or 'n'
	static bool getYesNo(std::string question){
		while(true){
			std::string user_entry;
			printf("%s (y/n)\t", question.c_str());
			std::cin >> user_entry;
			if (user_entry=="y"){
				return true;
			} else if (user_entry=="n"){
				return false;
			} else {
				printf("Unrecognized entry. ");
			}
		}
	}
};