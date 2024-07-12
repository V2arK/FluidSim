#version 460 core

layout(location = 0) in vec3 aPos;          // Vertex position attribute
layout(location = 1) in vec3 aColor;        // Vertex color attribute

out vec3 ourColor;                         // Output color to fragment shader

uniform mat4 model;                        // Model matrix to transform vertices to world space
uniform mat4 view;                         // View matrix to transform vertices to camera space
uniform mat4 projection;                   // Projection matrix to transform vertices to clip space
uniform vec3 spherePosition;               // Position of the sphere in world space

void main() {
    // Calculate the final position of the vertex by transforming it through model, view, and projection matrices
    gl_Position = projection * view * model * vec4(aPos + spherePosition, 1.0); 
    ourColor = aColor;                     // Pass the vertex color to the fragment shader
}
