# cs488 Project

## Overview

This project, titled "Position Based Dynamics Fluid Simulator," aims to develop a comprehensive and versatile fluid simulator using a Position-Based Dynamics (PBD) framework. By integrating Smoothed Particle Hydrodynamics (SPH) within the PBD framework and utilizing OpenGL for rendering, the simulator strives to achieve high fidelity in modeling fluid behavior in real-time. 

It is based on the OpenGL starter template provided by [Adrian Derstroff's OpenGL Starter](https://github.com/adrianderstroff/opengl-starter), which is released under the MIT license. The `opengl-starter` project offers a basic framework for compiling and managing various libraries required for OpenGL development.

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
    cmake . && make
    ```

2. **Run the Executable**:
   Navigate to the `bin` directory and execute the compiled binary:

   ```bash
    cd bin && ./*
    ```


## Main Project Structure

```plaintext
cs488/
├── assets/
│   └── shaders/
├── src/
│   ├── main.cpp
│   └── pbd.h
└── README.md
```

## Feature Detail

### Implementing the graphic engine using OpenGL
Firstly, since the starter code I used has only an rasterizer for one triangle, I re-write the entire `main.cpp`. The steps are rather similar to `A1`, with the only difference being in `A1`, the pixel buffer is populated by the CPU, then hand over to OpenGL vertex and fragment shader pipeline to pump out to the display; And in my current setup, the coordinates and the model of the spheres are hand over to OpenGL vertex and fragment shader pipeline, and the GPU handles the entiring rasterizing process.

I then add supports to movements using `WSAD` and camera operation with mouse, clipping and depth buffer, as well as the drawing of spheres, which conclude the basic rasterization using `OpenGL`.

![image](/screenshots/sc1.png)

![image](/screenshots/sc2.png)

### Fluid Simulation (Smoothed-particle dynamics)

The structure of `pbd.h` can be summarized as:

```
ParticleSystem
|
├── CubicSplineKernel
│   ├── Value()
│   ├── Gradient()
|
├── ParticleSystem
│   ├── Constructor and Destructor
│   ├── SetContainerSize(
│   ├── AddFluidBlock()
│   ├── clearParticle()
│   ├── Iterate()
│       ├── ResetAcceleration()
│       ├── SearchNeighbors()
│       ├── UpdateDensityAndPressure()
│       ├── UpdateViscosityAcceleration()
│       ├── UpdatePressureAcceleration()
│       ├── EulerIntegrate()
│       ├── ApplyBoundaryConditions()
```
