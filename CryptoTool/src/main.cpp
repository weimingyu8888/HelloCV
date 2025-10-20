#include "../include/Menu.h"
#include <iostream>

int main() {
 
    std::cout << "欢迎使用文本加密解密工具 (XOR)" << std::endl;
    std::cout << "使用说明：" << std::endl;
    std::cout << "- 基于XOR加密算法" << std::endl;
    std::cout << "- 加密和解密使用相同的密码" << std::endl;
    std::cout << "- 支持文本和文件的加密解密" << std::endl;
  
    
    try {
        Menu::handleUserInput();
    } catch (const std::exception& e) {
        std::cerr << "程序发生错误: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}