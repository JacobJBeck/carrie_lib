// Copyright 2016 Carrie Rebhuhn
#ifndef FILEIO_FILEOUT_H
#define FILEIO_FILEOUT_H

#pragma warning(disable:4996) // disables strtok deprecation warnings (portability)
#include "FileErrors.h"
#include <vector>
#include <errno.h>
#include <limits.h>
#include <stdlib.h>
#include <string.h>

#ifndef _WIN32
#include <sys/stat.h>
#else
#include <direct.h>
#endif // !WIN32


/*!
A static class to print to a file.
*/

class FileOut {
public:
    static void mkdir_p(std::string pathname) {
        char *tok = NULL;
        char *path = new char[pathname.length() + 1];
        strcpy(path, pathname.c_str());
        std::string tmp;

        tok = strtok(path, "/");

        // keep going until there are no tokens left
        while (tok)
        {
            // append the next token to the path
            tmp.append(tok);

            // Create the directory
            #ifdef WIN32
                mkdir(tmp.c_str());
            #else
                mkdir(tmp.c_str(),666);
            #endif
            
            // append a / to the path for the next token and get it
            tmp.append("/");

            tok = strtok(NULL, "/");
        }
        delete[]tok;
        delete[]path;
    }


    //! Output function for 1d vector, list, etc: anything that can be iterated over
    template<class T>
    static void print_vector(std::vector<T> &output, std::string file_name, bool overwrite = true, std::string separator = ",") {
        std::ofstream file;
        if (overwrite) file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
        else file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);

        if (!overwrite) file << '\n';

        if (file.is_open()) {
            print_vector(output, file, separator);
            file.close();
            printf("... Successfully wrote to file %s.\n", file_name.c_str());
        } else FileErrors::failed_file(file_name);
    }

    //! Output function for 2d vector, list, etc: anything that can be iterated over
    template<class T>
    static void print_vector(std::vector<std::vector<T> > &output, std::string file_name, bool overwrite = true, std::string separator = ",") {
        std::ofstream file;
        if (overwrite) file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
        else file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);

        if (!overwrite) file << '\n';

        if (file.is_open()) {
            for (std::vector<T> outer : output) {
                print_vector(outer, file, separator);
                file << "\n";
            }
            file.close();
            printf("... Successfully wrote to file %s.\n", file_name.c_str());
        } else FileErrors::failed_file(file_name);
    }


    //! Output function for 3d vector, list, etc: anything that can be iterated over
    template <class T>
    static void print_vector(std::vector<std::vector<std::vector<T> > > &output, std::string file_name, bool overwrite = true, std::string separator = ",") {
        std::ofstream file;
        if (overwrite) file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
        else file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);

        if (!overwrite) file << '\n';

        if (file.is_open()) {
            for (std::vector<std::vector<T> > outer : output) {
                print_vector(outer, file, separator);
                file << "\n";
            }
            file.close();
            printf("... Successfully wrote to file %s.\n", file_name.c_str());
        } else FileErrors::failed_file(file_name);
    }

    //! Print-to-file for a 1D container of pairs (also works with maps).
    template<class T>
    static void print_pair_container(T data, std::string file_name, bool overwrite = true, std::string separator = ",") {
        std::ofstream file;
        if (overwrite) file.open(file_name.c_str(), std::ofstream::out | std::ofstream::trunc);
        else file.open(file_name.c_str(), std::ofstream::out | std::ofstream::app);

        if (!overwrite) file << '\n';

        if (file.is_open()) {
            for (auto it : data) {
                file << it.first << separator << it.second << separator << "\n";
            }
            file.close();
            printf("... Successfully wrote to file %s.\n", file_name.c_str());
        } else
            FileErrors::failed_file(file_name);
    }

private:
    //! Helper function for 1- and 2-d file output
    template <class T>
    static void print_vector(std::vector<T> output, std::ofstream &file, std::string separator) {
        for (T outer : output) {
            file << outer << separator;
        }
    }

    //! Helper function for 3-d file output
    template <class T>
    static void print_vector(std::vector<std::vector<T> > output, std::ofstream &file, std::string separator) {
        for (std::vector<T> outer : output) {
            print_vector(outer, file, separator);
        }
    }
};
#endif  // !FILEIO_FILEOUT_H
