#include "../include/Crypto.h"

std::string Crypto::encrypt(const std::string& text, const std::string& password) {
    
    if (password.empty()) {
        return text;
    }
    
    std::string result = text;
    size_t passwordLength = password.length();
    
    
    for (size_t i = 0; i < result.length(); ++i) {
        
        result[i] = result[i] ^ password[i % passwordLength];
    }
    
    return result;
}

std::string Crypto::decrypt(const std::string& encryptedText, const std::string& password) {
    
    return encrypt(encryptedText, password);
}