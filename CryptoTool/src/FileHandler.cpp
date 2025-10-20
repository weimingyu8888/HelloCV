#include "../include/FileHandler.h"
#include "../include/Crypto.h"
#include <fstream>
#include <iostream>

std::string FileHandler::readFile(const std::string& filename) {
    std::ifstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "错误：无法打开文件 " << filename << std::endl;
        return "";
    }
    
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    file.close();
    return content;
}

bool FileHandler::writeFile(const std::string& filename, const std::string& content) {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
        std::cerr << "错误：无法创建文件 " << filename << std::endl;
        return false;
    }
    
    file.write(content.c_str(), content.length());
    file.close();
    return true;
}

bool FileHandler::encryptFile(const std::string& inputFile, const std::string& outputFile, const std::string& password) {
    std::cout << "正在读取文件: " << inputFile << std::endl;
    std::string content = readFile(inputFile);
    if (content.empty()) {
        std::cerr << "错误：文件内容为空或读取失败" << std::endl;
        return false;
    }
    
    std::cout << "正在加密文件..." << std::endl;
    std::string encryptedContent = Crypto::encrypt(content, password);
    
    std::cout << "正在写入加密文件: " << outputFile << std::endl;
    bool success = writeFile(outputFile, encryptedContent);
    
    if (success) {
        std::cout << "文件加密成功！" << std::endl;
    } else {
        std::cerr << "文件加密失败！" << std::endl;
    }
    
    return success;
}

bool FileHandler::decryptFile(const std::string& inputFile, const std::string& outputFile, const std::string& password) {
    std::cout << "正在读取加密文件: " << inputFile << std::endl;
    std::string encryptedContent = readFile(inputFile);
    if (encryptedContent.empty()) {
        std::cerr << "错误：文件内容为空或读取失败" << std::endl;
        return false;
    }
    
    std::cout << "正在解密文件..." << std::endl;
    std::string decryptedContent = Crypto::decrypt(encryptedContent, password);
    
    std::cout << "正在写入解密文件: " << outputFile << std::endl;
    bool success = writeFile(outputFile, decryptedContent);
    
    if (success) {
        std::cout << "文件解密成功！" << std::endl;
    } else {
        std::cerr << "文件解密失败！" << std::endl;
    }
    
    return success;
}