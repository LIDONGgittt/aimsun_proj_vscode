## Aimsun project on Windows platform with C++, CMake and VSCode

### Dependencies:
- C++
- CMake
- Aimsun

### Local configuration
1. Install all dependencies.
    1. C++ toolchain: [MSYS2](https://www.msys2.org/) (Recommended) or MinGW
    1. Cmake: [Download page](https://cmake.org/download/), Windows x64 Installer
    1. VSCode: [Download page](https://code.visualstudio.com/download)
        - Useful [extensions](https://code.visualstudio.com/docs/editor/extension-marketplace) after installation:
            - VS extension [C/C++](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools)
            - VS extension [CMake](https://marketplace.visualstudio.com/items?itemName=twxs.cmake)
            - VS extension [CMake Tools](https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools)  



1. In .vscode/settings.json, modify "cmake.cmakePath" to the installed path of cmake.exe on your local machine.

1. Config and Build with CMake Tools. You may have multiple modified api files under `./src` directory. Every cxx/cpp file will be compiled seperately. Compiled dynamic library will be in the `./lib` folder.

