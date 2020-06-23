#include <boost/filesystem.hpp>

#include <iomanip>
#include <iostream>

namespace fs = boost::filesystem;

const int width = 26;

#define PRINT_METHOD(x) \
  std::cout << std::left << std::setw(width) << #x << path.x() << std::endl
#define PRINT_FUNCTION(x) \
  std::cout << std::left << std::setw(width) << #x << fs::x(path) << std::endl

int main(int argCount, char* argList[]) {
  for (int i = 1; i < argCount; i++) {
    fs::path path(argList[i]);
    std::cout << std::left << std::setw(width) << "path" << path << std::endl;
    PRINT_METHOD(extension);
    PRINT_METHOD(filename);
    PRINT_METHOD(parent_path);
    PRINT_METHOD(relative_path);
    PRINT_METHOD(root_directory);
    PRINT_METHOD(root_name);
    PRINT_METHOD(root_path);
    PRINT_METHOD(stem);
    PRINT_METHOD(string);
    PRINT_METHOD(empty);
    PRINT_METHOD(is_complete);
    PRINT_METHOD(has_root_name);
    PRINT_METHOD(has_root_directory);
    PRINT_METHOD(has_root_path);
    PRINT_METHOD(has_relative_path);
    PRINT_METHOD(has_filename);
    PRINT_METHOD(has_parent_path);
    PRINT_FUNCTION(exists);
    PRINT_FUNCTION(is_regular_file);
    PRINT_FUNCTION(absolute);
    PRINT_FUNCTION(canonical);
    //PRINT_METHOD(file_string);
    //PRINT_METHOD(directory_string);
    //PRINT_METHOD(external_file_string);
    //PRINT_METHOD(external_directory_string);
    //std::cout << "relative_path():     " << path.relative_path() << std::endl;
    //std::cout << "root_path():         " << path.root_path() << std::endl;
    //std::cout << "parent_path():       " << path.parent_path() << std::endl;
    //std::cout << "filename():          " << path.filename() << std::endl;
    //std::cout << "string():            " << path.string() << std::endl;
    //std::cout << "file_string():       " << path.file_string() << std::endl;
    //std::cout << "directory_string():  " << path.directory_string() << std::endl;
    //std::cout << "external_file_string(): " << path.e
    std::cout << std::endl;
  }
}
