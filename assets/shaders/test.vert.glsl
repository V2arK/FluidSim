#version 460 core

// Input attributes from the vertex buffer
layout(location = 0) in vec3 aPos;  // The position variable has attribute position 0
layout(location = 1) in vec3 aColor; // The color variable has attribute position 1

// Output to the fragment shader
out vec3 ourColor; // Output variable to pass the color to the fragment shader

// Uniform variables for transformation matrices
uniform mat4 model;      // Model matrix: used to transform the vertices from local space to world space
uniform mat4 view;       // View matrix: used to transform the vertices from world space to camera space
uniform mat4 projection; // Projection matrix: used to project the 3D coordinates into the 2D space of the screen

void main() {
    // Calculate the position of the vertex by applying the model, view, and projection matrices
    // to the input position (aPos). The multiplication order is important: projection * view * model.
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    
    // Pass the input color to the fragment shader
    ourColor = aColor;
}
