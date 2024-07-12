#version 460 core

in vec3 ourColor;                          // Input color from vertex shader
out vec4 FragColor;                        // Output color of the fragment

void main() {
    FragColor = vec4(ourColor, 1.0f);      // Set the fragment color to the input color with full opacity
}
