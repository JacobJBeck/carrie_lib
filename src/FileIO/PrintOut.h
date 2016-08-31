#pragma once
#include <vector>

/*!
A static class to print to the screen.
*/

class PrintOut{
public:
	//! Takes in address of first element of 1D array and prints out, optionally labeled.
	template <typename DataType, size_t N>
	static void print_array(DataType data[], std::string label=""){
		printf("Showing %s:\n\n", label.c_str());
		for (int i=0; i<N; i++){
			std::cout << data[i] << ", ";
		}
		printf("\n\n");
	}

	//! Takes in address of first element of 2D array and prints out, optionally labeled 
	template <typename DataType, size_t N, size_t M>
	static void print_array(DataType array_first[N][M], std::string label=""){
		printf("Showing %s:\n\n", label.c_str());
		for (int i=0; i<N; i++){
			for (int j=0; j<M; j++){
				std::cout << array_first[i][j] << ", ";
			}
			printf("\n");
		}
		printf("\n\n");
	}

	template<class T>
	static void print_vector(std::vector<T> data,std::string label="",std::string separator=","){
		if (label!="") std::cout << label << ":" << std::endl;
		for (T it : data)
			std::cout << it << separator;
		std::cout << std::endl;
	}

	//! Prints a 2D stl container, given a container and an optional label and separator.
	template<class T>
	static void print_vector(std::vector<std::vector<T> > data,std::string label="",std::string separator=","){
		if (label!="") std::cout << label << ":" << std::endl;
		for (std::vector<T> outer : data){
			for (T inner : outer){
				std::cout << inner << separator;
			}
			std::cout << std::endl;
		}
		std::cout << std::endl;
	}
};