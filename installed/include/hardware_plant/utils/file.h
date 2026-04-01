#pragma once

#include <string>
#include <iostream>
#include <fstream>

namespace leju {
namespace hw {

class FileUtils {
public:
    /**
     * @brief Check if a file or directory exists at the given path
     *
     * This function checks whether the specified file or directory path exists
     * on the filesystem. It can be used to verify file existence before
     * performing read/write operations.
     *
     * @param filepath [in] Path to the file or directory to check
     * @return true if the path exists, false if the path does not exist or is inaccessible
     */
    static bool Exists(const std::string &filepath);

    /**
     * @brief Read entire file content into a string using std::ifstream
     *
     * This function opens the specified file in read mode and reads its entire content
     * into the provided output string. The implementation uses std::ifstream with
     * std::stringstream to efficiently read the complete file content.
     *
     * @param file_path [in] Path to the file to be read
     * @param out_content [out] String that will be cleared and populated with the file's entire content
     * @return true if the file was successfully opened and read, false if the file cannot be opened or doesn't exist
     */
    static bool ReadToString(const std::string& file_path, std::string& out_content);

    /**
     * @brief Check if the given path is a directory
     *
     * This function checks whether the specified path points to a directory
     * on the filesystem. It returns false if the path does not exist,
     * is inaccessible, or is a regular file.
     *
     * @param dirpath [in] Path to the directory to check
     * @return true if the path exists and is a directory, false otherwise
     */
    static bool IsDirectory(const std::string &dirpath);

};
} // namespace hw
} // namespace leju 