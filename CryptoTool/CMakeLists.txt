cmake_minimum_required(VERSION 3.10)
project(CryptoTool)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 包含头文件目录
include_directories(include)

# 添加可执行文件
add_executable(CryptoTool
    src/main.cpp
    src/Crypto.cpp
    src/FileHandler.cpp
    src/Menu.cpp
)

# 设置输出目录
set_target_properties(CryptoTool PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)