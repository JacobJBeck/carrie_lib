#ifndef SRC_FILEIO_CONVERSION_H_
#define SRC_FILEIO_CONVERSION_H_

#include <vector>

typedef std::vector<std::string> string_matrix1d;


/*!
A static class for converting between data types.
*/

class Conversion {
 public:
    //! Converts an element from one type to another using stringstream.
    template <typename OutputType, typename InputType>
    static OutputType convert_element(const InputType &input) {
        std::stringstream convert;
        convert << input;
        OutputType output;
        convert >> output;
        return output;
    }

    /*!
    Converts a vector to a vector of another type using stringstream.
    Requires call of type convert_vector<T1,T2>.
    */
    template <typename OutputType, typename InputType>
    static std::vector<std::vector<OutputType> >
        convert_vector(const std::vector<std::vector<InputType> > &f) {
        std::vector<std::vector<OutputType> > var(f.size());
        for (unsigned int i = 0; i < f.size(); i++) {
            var[i] = std::vector<OutputType>(f[i].size());
            for (unsigned int j = 0; j < f[i].size(); j++) {
                var[i][j] = convert_element<OutputType>(f[i][j]);
            }
        }
        return var;
    }

    //! Divides a string by 'separator'
    static string_matrix1d divide(std::string my_string,
        const std::string &separator) {
        string_matrix1d divided;
        while (my_string.find(separator) != std::string::npos) {
            // Find the separator
            size_t found = my_string.find(separator);
            // Cut at separator
            divided.push_back(my_string.substr(0, found));
            // Remove to separator
            my_string.erase(my_string.begin(), my_string.begin() + found + 1);
        }
        divided.push_back(my_string);
        return divided;
    }

};
#endif  // SRC_FILEIO_CONVERSION_H_
