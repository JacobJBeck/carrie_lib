// Copyright 2016 Carrie Rebhuhn
#ifndef FILEIO_FILEIN_H_
#define FILEIO_FILEIN_H_

#include "FileErrors.h"
#include "Conversion.h"
#include <map>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#define STRING_UNINITIALIZED "string_uninitialized"

typedef std::vector<double> matrix1d;
typedef std::vector<matrix1d> matrix2d;
typedef std::vector<string_matrix1d> string_matrix2d;
typedef std::map<std::string, double> VarMap;

/*!
A namespace that handles variable setting and standard file reading operations.
*/

namespace easyio {
bool file_exists(std::string filename);

//! Read in pair type objects
template<typename T>
std::vector<T> read_pairs(std::string file_name,
    std::string separator = STRING_UNINITIALIZED) {
    std::vector<std::vector<int> >  m = read2<int>(file_name, separator);
    std::vector<T> var;
    for (std::vector<int> i : m) {
        if (i.size() != 2) {
            FileErrors::not_pair(file_name);
        } else {
            var.push_back(T(i[0], i[1]));
        }
    }
    return var;
}

//! Infer the separator from the file type
static std::string detect_separator(std::string file_name) {
    std::vector<std::string> divided = Conversion::divide(file_name, ".");
    if (divided.back() == "csv") {
        return ",";
    } else if (divided.back() == "xls") {
        return "\t";
    } else {
        FileErrors::unrecognized_extension(divided.back());
        return "";
    }
}

//! Reads in a 2D data file
template <typename DataType=double>
static std::vector<std::vector<DataType> > read2(std::string file_name,
    std::string separator = STRING_UNINITIALIZED) {
    if (separator == STRING_UNINITIALIZED)
        separator = detect_separator(file_name);

    std::ifstream file(file_name.c_str());
    if (!file.is_open()) {
        FileErrors::failed_file(file_name);
        return std::vector<std::vector<DataType> >();
    }

    std::string value;
    string_matrix2d file_matrix;

    while (file.good()) {
        getline(file, value);
        std::istringstream iss(value);
        std::string word;
        string_matrix1d line;
        while (getline(iss, word, *separator.c_str()))
            line.push_back(word);
        if (line.size())
            file_matrix.push_back(line);
    }
    file.close();

    std::vector<std::vector<DataType> > d
        = Conversion::convert_vector<DataType>(file_matrix);

    printf("... Successfully read in file %s.\n", file_name.c_str());
    return d;
}

//! Reads variables (double) from a file to access by name (string)
static VarMap read_variable_file(std::string file_name);
}  // namespace FileIO
#endif  // FILEIO_FILEIN_H_
