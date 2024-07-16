#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>

#include <stdio.h>
#include <stddef.h>
#include <cstddef>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "fbh.h"

// Camera parameters for setting up the view matrix
glm::vec3 cameraPos = glm::vec3(0.0f, -0.2f, 2.0f);	  // Camera position
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f); // Camera direction
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);	  // Camera up vector
float yaw = -90.0f;									  // Initial yaw angle
float pitch = 0.0f;									  // Initial pitch angle
float lastX = 960.0f, lastY = 540.0f;				  // Last recorded mouse position
bool firstMouse = true;								  // Flag for initial mouse movement
float fov = 45.0f;									  // Field of view for the projection matrix
float deltaTime = 0.0f;								  // Time between current frame and last frame
float lastFrame = 0.0f;								  // Time of last frame

// Movement speeds for camera movement
float movementSpeed = 10.0f;
float mouseSensitivity = 0.1f;

// Define parameters for the sphere
int sectorCount = 10;
int stackCount = 5;

// Particle system
ParticleSystem2D ps2;
ParticleSystem3D ps3;

// Global vector to store sphere indices for rendering
std::vector<GLuint> sphereIndices;

/* ----------------- Call Back Func -------------------- */

// Callback function to handle GLFW errors
static void errorCallback(int error, const char *description)
{
	std::cerr << description;
}

// Callback function to handle key input for camera movement and window closing
static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
{
	// Close window when ESC is pressed
	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
		glfwSetWindowShouldClose(window, GL_TRUE);

	float cameraSpeed = movementSpeed * deltaTime; // Adjust camera speed based on frame time

	/*
	// Move camera in the direction it's facing
	if (key == GLFW_KEY_W)
		cameraPos += cameraSpeed * cameraFront;
	if (key == GLFW_KEY_S)
		cameraPos -= cameraSpeed * cameraFront;
	// Strafe camera left and right
	if (key == GLFW_KEY_A)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (key == GLFW_KEY_D)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	*/
	if (key == GLFW_KEY_C)
	{
		ps2.clearParticle();
		ps3.clearParticle();
	}
		
}

// Callback function for mouse button input (not used here but defined for completeness)
static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {}

// Callback function to handle mouse movement for adjusting camera orientation
static void mousePositionCallback(GLFWwindow *window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;		// Initialize lastX with the first mouse x position
		lastY = ypos;		// Initialize lastY with the first mouse y position
		firstMouse = false; // Set firstMouse to false after initialization
	}

	float xoffset = xpos - lastX; // Calculate horizontal offset
	float yoffset = lastY - ypos; // Calculate vertical offset (reversed since y-coordinates range from bottom to top)
	lastX = xpos;				  // Update lastX with current x position
	lastY = ypos;				  // Update lastY with current y position

	xoffset *= mouseSensitivity; // Adjust offset based on mouse sensitivity
	yoffset *= mouseSensitivity;

	yaw += xoffset;	  // Update yaw angle
	pitch += yoffset; // Update pitch angle

	// Constrain the pitch angle to avoid gimbal lock
	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	// Calculate the new front vector based on updated yaw and pitch
	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(front);
}

// Callback function to handle window resizing
static void framebufferSizeCallback(GLFWwindow *window, int w, int h)
{
	glViewport(0, 0, w, h); // Update the OpenGL viewport to match the new window size
}

/* ----------------- Initializer -------------------- */

// Function to initialize GLFW and create a window with specified dimensions and title
GLFWwindow *initialize(int width, int height, const std::string &title)
{
	GLFWwindow *window;
	glfwSetErrorCallback(errorCallback); // Set the error callback function

	// Initialize GLFW library
	if (!glfwInit())
		exit(EXIT_FAILURE);

	// Set GLFW window hints for OpenGL version and profile
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Create a windowed mode window and its OpenGL context
	window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
	if (!window)
	{
		std::cerr << "Failed to open GLFW window.\n";
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window); // Make the window's context current

	// Load all OpenGL function pointers using GLAD
	if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress))
	{
		std::cerr << "Failed to initialize GLAD.\n";
		exit(EXIT_FAILURE);
	}

	// Print the renderer and OpenGL version
	const GLubyte *renderer = glGetString(GL_RENDERER);
	const GLubyte *version = glGetString(GL_VERSION);
	std::cout << "Renderer: " << renderer << '\n';
	std::cout << "OpenGL version supported: " << version << '\n';

	// Set GLFW callback functions for input handling
	glfwSetKeyCallback(window, keyCallback);
	glfwSetMouseButtonCallback(window, mouseButtonCallback);
	// glfwSetCursorPosCallback(window, mousePositionCallback);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

	// Set the clear color for the window
	glClearColor(0.3f, 0.3f, 0.3f, 0.0f);

	// Enable depth testing for correct rendering of 3D objects
	glEnable(GL_DEPTH_TEST);

	// Hide the mouse cursor and capture it within the window
	// glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	return window;
}

/* ----------------- Sphere generation -------------------- */

// Struct to hold vertex data (position and color)
struct Vertex
{
	glm::vec3 pos;
	glm::vec3 color;
};

// Function to generate vertices for a sphere
std::vector<Vertex> generateSphere(float radius, int sectorCount, int stackCount)
{
	std::vector<Vertex> vertices;
	float x, y, z, xy;						   // Variables for vertex positions
	float sectorStep = 2 * M_PI / sectorCount; // Step size for sectors (longitude)
	float stackStep = M_PI / stackCount;	   // Step size for stacks (latitude)
	float sectorAngle, stackAngle;

	// Loop through stacks and sectors to generate vertices
	for (int i = 0; i <= stackCount; ++i)
	{
		stackAngle = M_PI / 2 - i * stackStep; // Calculate current stack angle
		xy = radius * cosf(stackAngle);		   // Calculate x and y based on stack angle
		z = radius * sinf(stackAngle);		   // Calculate z based on stack angle

		for (int j = 0; j <= sectorCount; ++j)
		{
			sectorAngle = j * sectorStep; // Calculate current sector angle

			x = xy * cosf(sectorAngle); // Calculate x position
			y = xy * sinf(sectorAngle); // Calculate y position
			vertices.push_back({glm::vec3(x, y, z), glm::vec3((x + radius) / (2 * radius), (y + radius) / (2 * radius), (z + radius) / (2 * radius))});
		}
	}

	return vertices;
}

// Function to generate indices for drawing sphere triangles
std::vector<GLuint> generateIndices(int sectorCount, int stackCount)
{
	std::vector<GLuint> indices;
	int k1, k2; // Indices for stacks and sectors

	// Loop through stacks and sectors to generate indices
	for (int i = 0; i < stackCount; ++i)
	{
		k1 = i * (sectorCount + 1); // Index for current stack
		k2 = k1 + sectorCount + 1;	// Index for next stack

		for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
		{
			if (i != 0)
			{
				indices.push_back(k1); // First triangle
				indices.push_back(k2);
				indices.push_back(k1 + 1);
			}

			if (i != (stackCount - 1))
			{
				indices.push_back(k1 + 1); // Second triangle
				indices.push_back(k2);
				indices.push_back(k2 + 1);
			}
		}
	}

	return indices;
}

// Function to create buffers for the sphere
GLuint createBuffersForSphere(float radius, int sectorCount, int stackCount)
{
	std::vector<Vertex> vertices = generateSphere(radius, sectorCount, stackCount); // Generate sphere vertices
	sphereIndices = generateIndices(sectorCount, stackCount);						// Generate sphere indices

	GLuint vao, vbo, ebo;
	glGenVertexArrays(1, &vao); // Create a vertex array object
	glGenBuffers(1, &vbo);		// Create a vertex buffer object
	glGenBuffers(1, &ebo);		// Create an element buffer object

	glBindVertexArray(vao); // Bind the vertex array object

	glBindBuffer(GL_ARRAY_BUFFER, vbo);															   // Bind the vertex buffer object
	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW); // Copy vertex data to buffer

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);																		 // Bind the element buffer object
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(GLuint), &sphereIndices[0], GL_STATIC_DRAW); // Copy index data to buffer

	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)(offsetof(Vertex, pos)));	// Specify vertex position attribute
	glEnableVertexAttribArray(0);																		// Enable the vertex position attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)(offsetof(Vertex, color))); // Specify vertex color attribute
	glEnableVertexAttribArray(1);																		// Enable the vertex color attribute

	glBindVertexArray(0); // Unbind the vertex array object

	return vao;
}

/* ----------------- Shader related -------------------- */

// Function to compile a shader from a file
GLuint compileShader(const std::string &path, GLenum shaderType)
{
	std::ifstream filestream(path);																			  // Open the shader file
	std::string shaderSource((std::istreambuf_iterator<char>(filestream)), std::istreambuf_iterator<char>()); // Read the shader source

	GLuint shaderHandle = glCreateShader(shaderType);			// Create a shader object
	const char *shaderSourcePtr = shaderSource.c_str();			// Get a pointer to the shader source
	glShaderSource(shaderHandle, 1, &shaderSourcePtr, nullptr); // Set the shader source
	glCompileShader(shaderHandle);								// Compile the shader

	int success;
	char infoLog[512];
	glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &success); // Check for compilation errors
	if (!success)
	{
		glGetShaderInfoLog(shaderHandle, 512, nullptr, infoLog); // Get the error log
		std::cerr << "Error while compiling shader\n"
				  << infoLog << std::endl;
	}

	return shaderHandle;
}

// Function to create a shader program from vertex and fragment shaders
GLuint createShaderProgram(const std::string &vertexShaderPath, const std::string &fragmentShaderPath)
{
	GLuint vertexShader = compileShader(vertexShaderPath, GL_VERTEX_SHADER);	   // Compile the vertex shader
	GLuint fragmentShader = compileShader(fragmentShaderPath, GL_FRAGMENT_SHADER); // Compile the fragment shader

	GLuint shaderProgram = glCreateProgram();	   // Create a shader program
	glAttachShader(shaderProgram, vertexShader);   // Attach the vertex shader
	glAttachShader(shaderProgram, fragmentShader); // Attach the fragment shader
	glLinkProgram(shaderProgram);				   // Link the shaders into the program

	int success;
	char infoLog[512];
	glGetShaderiv(shaderProgram, GL_LINK_STATUS, &success); // Check for linking errors
	if (!success)
	{
		glGetShaderInfoLog(shaderProgram, 512, nullptr, infoLog); // Get the error log
		std::cerr << "Error while linking shaders\n"
				  << infoLog << std::endl;
	}

	glDeleteShader(vertexShader);	// Delete the vertex shader as it's no longer needed
	glDeleteShader(fragmentShader); // Delete the fragment shader as it's no longer needed

	return shaderProgram;
}

// Function to render the spheres
void render(GLuint shaderProgram, GLuint vao, const std::vector<glm::vec2> &particles)
{
	glUseProgram(shaderProgram); // Use the shader program

	// Set up the view matrix based on the camera's position, direction, and up vector
	glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	// Set up the projection matrix with perspective projection
	glm::mat4 projection = glm::perspective(glm::radians(fov), 1920.0f / 1080.0f, 0.1f, 100.0f);

	// Pass the view matrix to the shader
	GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

	// Pass the projection matrix to the shader
	GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

	glBindVertexArray(vao); // Bind the vertex array object

	// Loop through each sphere position and render the sphere
	for (const auto &p : particles)
	{
		glm::vec3 pos(p, 0.0f);
		glm::mat4 model = glm::mat4(1.0f); // Create the model matrix
		GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model)); // Pass the model matrix to the shader

		GLint spherePosLoc = glGetUniformLocation(shaderProgram, "spherePosition");
		glUniform3fv(spherePosLoc, 1, glm::value_ptr(pos)); // Pass the sphere position to the shader

		glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0); // Draw the sphere using the index buffer
	}
}

// Function to render the spheres
void render(GLuint shaderProgram, GLuint vao, const std::vector<glm::vec3> &particles)
{
	glUseProgram(shaderProgram); // Use the shader program

	// Set up the view matrix based on the camera's position, direction, and up vector
	glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
	// Set up the projection matrix with perspective projection
	glm::mat4 projection = glm::perspective(glm::radians(fov), 1920.0f / 1080.0f, 0.1f, 100.0f);

	// Pass the view matrix to the shader
	GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

	// Pass the projection matrix to the shader
	GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

	glBindVertexArray(vao); // Bind the vertex array object

	// Loop through each sphere position and render the sphere
	for (const auto &p : particles)
	{
		glm::mat4 model = glm::mat4(1.0f); // Create the model matrix
		GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model)); // Pass the model matrix to the shader

		GLint spherePosLoc = glGetUniformLocation(shaderProgram, "spherePosition");
		glUniform3fv(spherePosLoc, 1, glm::value_ptr(p)); // Pass the sphere position to the shader

		glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0); // Draw the sphere using the index buffer
	}
}

// Function to clean up resources
void cleanup(GLFWwindow *window, GLuint &shaderProgram, GLuint &vao)
{
	glDeleteProgram(shaderProgram); // Delete the shader program
	glDeleteVertexArrays(1, &vao);	// Delete the vertex array object

	glfwDestroyWindow(window); // Destroy the GLFW window
	glfwTerminate();		   // Terminate GLFW
}

/* ----------------- Main Loop -------------------- */
int main()
{
	// Initialize GLFW and create a window
	GLFWwindow *window = initialize(1920, 1080, "Fluid Simulation");

	// Create the shader program
	GLuint shaderProgram = createShaderProgram(
		ASSETS_PATH "/shaders/test.vert.glsl",
		ASSETS_PATH "/shaders/test.frag.glsl");

	// Particle system initialization
	// ps2.SetContainerSize(glm::vec2(-1.0, -1.0), glm::vec2(1.0, 0.5));
	// ps2.AddFluidBlock(glm::vec2(-0.2, -0.2), glm::vec2(0.2, 0.2));

	ps3.SetContainerSize(glm::vec3(-1, -1, -1), glm::vec3(1, 0.5, 0));
	//ps3.AddFluidBlock(glm::vec3(0, 0, -0.5), glm::vec3(0.1, 0.1, -0.4), glm::vec3(0, -9.8f, 0));
	//ps3.AddFluidBlock(glm::vec3(-0.25, 0.9, -0.45), glm::vec3(0.25, 0.95, -0.55), glm::vec3(0, -9.8f, 0));
	// Create buffers for the sphere
	GLuint vao = createBuffersForSphere(PARTICLE_RADIUS + 0.002f, sectorCount, stackCount);

	// Main render loop
	while (!glfwWindowShouldClose(window))
	{
		// Calculate delta time
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// Clear the color and depth buffers
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// ps2.AddFluidBlock(glm::vec2(-0.03f, 0.47f), glm::vec2(0.03f, 0.49f), glm::vec2(0.0f, 10.0f), 0.02f);
		// update particles
		// ps2.Iterate();

		ps3.AddFluidBlock(glm::vec3(0, 0, -0.46), glm::vec3(0.1, 0.03, -0.4), glm::vec3(0, -9.8f, 0), 0.02f);
		ps3.Iterate();

		// Render the spheres
		//render(shaderProgram, vao, ps2.particlePositions_);
		render(shaderProgram, vao, ps3.particlePositions_);

		// Swap the front and back buffers
		glfwSwapBuffers(window);
		glfwPollEvents(); // Poll for and process events
	}

	// Clean up resources
	cleanup(window, shaderProgram, vao);

	// Program exits successfully
	exit(EXIT_SUCCESS);
}
