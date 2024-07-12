# cs488 Project

## Overview

This project is based on the OpenGL starter template provided by [Adrian Derstroff's OpenGL Starter](https://github.com/adrianderstroff/opengl-starter), which is released under the MIT license. The `opengl-starter` project offers a basic framework for compiling and managing various libraries required for OpenGL development.

The initial codebase includes a simple `main.cpp` that rasterizes a basic triangle, similar to the example provided in Assignment 1 (A1).

## Prerequisites

Before you begin, ensure you have met the following requirements:
- **CMake**: Version 3.14 or later
- **Python**: Any recent version
- **OpenGL libraries**: GLFW3, GLAD, and GLM

For specific installation instructions, refer to the [OpenGL Starter README](https://github.com/adrianderstroff/opengl-starter).

## Compilation Instructions

### Using Visual Studio Code + Terminal

1.  **Compile the Code**:
    ```bash
    cmake . && make && cd bin && ./*
    ```

2. **Run the Executable**:
    - Navigate to the `bin` directory:
        ```bash
        cd bin
        ```
    - Execute the compiled binary:
        ```bash
        ./*
        ```

## Project Structure

```plaintext
cs488/
├── assets/
│   └── shaders/
├── cmake/
├── src/
│   ├── main.cpp
│   └── ...
├── .gitignore
├── CMakeLists.txt
├── LICENSE
└── README.md
```