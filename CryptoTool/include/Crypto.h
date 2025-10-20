#ifndef CRYPTO_H
#define CRYPTO_H

#include <string>

class Crypto {
public:
    static std::string encrypt(const std::string& text, const std::string& password);
    static std::string decrypt(const std::string& encryptedText, const std::string& password);
};

#endif