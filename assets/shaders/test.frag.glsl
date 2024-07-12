#version 460 core

// Input from the vertex shader
in vec3 ourColor; // The input color passed from the vertex shader

// Output to the framebuffer
out vec4 FragColor; // The final color output of the fragment

void main() {
    // Set the output color of the fragment to the input color, and set alpha to 1.0 (opaque)
    FragColor = vec4(ourColor, 1.0);
}
