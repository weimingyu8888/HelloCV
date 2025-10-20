#ifndef FILEHANDLER_H
#define FILEHANDLER_H

#include <string>

class FileHandler {
public:
    static std::string readFile(const std::string& filename);
    static bool writeFile(const std::string& filename, const std::string& content);
    static bool encryptFile(const std::string& inputFile, const std::string& outputFile, const std::string& password);
    static bool decryptFile(const std::string& inputFile, const std::string& outputFile, const std::string& password);
};

#endif