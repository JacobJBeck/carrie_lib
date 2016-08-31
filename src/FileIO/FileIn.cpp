#include "FileIn.h"

namespace easyio {
bool file_exists(std::string filename) {
    std::ifstream myfile(filename);
    bool exists = myfile.good();
    myfile.close();
    return exists;
}

//! Reads variables (double) from a file to access by name (string)
static VarMap read_variable_file(std::string file_name) {
    string_matrix2d raw_data = read2<std::string>(file_name);
    VarMap  processed_data;
    for (string_matrix1d &r : raw_data) {
        auto p = std::make_pair(r[0], std::stod(r[1].c_str()));
        processed_data.insert(p);
    }
    return processed_data;
}
}