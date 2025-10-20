#ifndef MENU_H
#define MENU_H

#include <string>

class Menu {
public:
    static void showMainMenu();
    static void handleUserInput();
    static void textEncryptionMenu();
    static void textDecryptionMenu();
    static void fileEncryptionMenu();
    static void fileDecryptionMenu();
    static std::string getInput(const std::string& prompt);
    static void clearInputBuffer();
};

#endif