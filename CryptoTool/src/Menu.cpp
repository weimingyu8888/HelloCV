#include "../include/Menu.h"
#include "../include/Crypto.h"
#include "../include/FileHandler.h"
#include <iostream>
#include <limits>

void Menu::showMainMenu() {
    std::cout << "文本加密解密工具 (XOR加密)" << std::endl;
    std::cout << "1. 文本加密" << std::endl;
    std::cout << "2. 文本解密" << std::endl;
    std::cout << "3. 文件加密" << std::endl;
    std::cout << "4. 文件解密" << std::endl;
    std::cout << "5. 退出程序" << std::endl;
    std::cout << "请选择功能 (1-5): ";
}

void Menu::handleUserInput() {
    int choice;
    
    while (true) {
        showMainMenu();
        std::cin >> choice;
        clearInputBuffer();  
        
        switch (choice) {
            case 1:
                textEncryptionMenu();
                break;
            case 2:
                textDecryptionMenu();
                break;
            case 3:
                fileEncryptionMenu();
                break;
            case 4:
                fileDecryptionMenu();
                break;
            case 5:
                std::cout << "bye" << std::endl;
                return;
            default:
                std::cout << "请输入1-5之间的数字" << std::endl;
        }
        
       
        std::cout << "按回车键继续...";
        std::cin.get();
    }
}

void Menu::textEncryptionMenu() {
    std::cout << "\n 文本加密" << std::endl;
    
    std::string text = getInput("请输入要加密的文本: ");
    if (text.empty()) {
        std::cout << "文本不能为空！" << std::endl;
        return;
    }
    
    std::string password = getInput("请输入加密密码: ");
    if (password.empty()) {
        std::cout << "密码不能为空！" << std::endl;
        return;
    }
    
    std::cout << "\n正在加密..." << std::endl;
    std::string encryptedText = Crypto::encrypt(text, password);
    
    std::cout << "加密成功！" << std::endl;
    std::cout << "原始文本: " << text << std::endl;
    std::cout << "加密结果: " << encryptedText << std::endl;
    
  
    std::cout << std::endl;
}

void Menu::textDecryptionMenu() {
    std::cout << "\n文本解密" << std::endl;
    
    std::string encryptedText = getInput("请输入要解密的文本: ");
    if (encryptedText.empty()) {
        std::cout << "加密文本不能为空！" << std::endl;
        return;
    }
    
    std::string password = getInput("请输入解密密码: ");
    if (password.empty()) {
        std::cout << "密码不能为空！" << std::endl;
        return;
    }
    
    std::cout << "\n正在解密..." << std::endl;
    std::string decryptedText = Crypto::decrypt(encryptedText, password);
    
    std::cout << "解密成功！" << std::endl;
    std::cout << "解密结果: " << decryptedText << std::endl;
}

void Menu::fileEncryptionMenu() {
    std::cout << "\n--- 文件加密 ---" << std::endl;
    
    std::string inputFile = getInput("请输入要加密的文件名: ");
    if (inputFile.empty()) {
        std::cout << "错误：文件名不能为空！" << std::endl;
        return;
    }
    
    std::string outputFile = getInput("请输入加密后的文件名: ");
    if (outputFile.empty()) {
        std::cout << "错误：输出文件名不能为空！" << std::endl;
        return;
    }
    
    std::string password = getInput("请输入加密密码: ");
    if (password.empty()) {
        std::cout << "错误：密码不能为空！" << std::endl;
        return;
    }
    
    if (FileHandler::encryptFile(inputFile, outputFile, password)) {
        std::cout << "✓ 文件加密完成！" << std::endl;
        std::cout << "加密文件: " << outputFile << std::endl;
    }
}

void Menu::fileDecryptionMenu() {
    std::cout << "\n--- 文件解密 ---" << std::endl;
    
    std::string inputFile = getInput("请输入要解密的文件名: ");
    if (inputFile.empty()) {
        std::cout << "错误：文件名不能为空！" << std::endl;
        return;
    }
    
    std::string outputFile = getInput("请输入解密后的文件名: ");
    if (outputFile.empty()) {
        std::cout << "错误：输出文件名不能为空！" << std::endl;
        return;
    }
    
    std::string password = getInput("请输入解密密码: ");
    if (password.empty()) {
        std::cout << "错误：密码不能为空！" << std::endl;
        return;
    }
    
    if (FileHandler::decryptFile(inputFile, outputFile, password)) {
        std::cout << "✓ 文件解密完成！" << std::endl;
        std::cout << "解密文件: " << outputFile << std::endl;
    }
}

std::string Menu::getInput(const std::string& prompt) {
    std::string input;
    std::cout << prompt;
    std::getline(std::cin, input);
    return input;
}

void Menu::clearInputBuffer() {
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}