This project, titled "Position Based Dynamics Fluid Simulator," aims to develop a comprehensive and versatile fluid simulator using a Position-Based Dynamics (PBD) framework. By integrating Smoothed Particle Hydrodynamics (SPH) within the PBD framework and utilizing OpenGL for rendering, the simulator strives to achieve high fidelity in modeling fluid behavior in real-time. 
It is based on the OpenGL starter template provided by [Adrian Derstroff's OpenGL Starter](https://github.com/adrianderstroff/opengl-starter), which is released under the MIT license. The `opengl-starter` project offers a basic framework for compiling and managing various libraries required for OpenGL development.
    cmake . && make
   Navigate to the `bin` directory and execute the compiled binary:

   ```bash
    cd bin && ./*
    ```

```

## Development Log

### Add implementing the graphic engine using OpenGL
Firstly, I re-write most of the `main.cpp` functions. I add supports to movements using `WSAD` and camera operation with mouse, clipping and depth buffer, as well as the drawing of spheres:

![image](/screenshots/sc1.png)

![image](/screenshots/sc2.png)

```diff
diff --git a/src/main.cpp b/src/main.cpp
index 21579e8..6596ab5 100644
--- a/src/main.cpp
+++ b/src/main.cpp
@@ -2,215 +2,375 @@
 #include <iostream>
 #include <cstdlib>
 #include <string>
+#include <vector>
 
 #include <glad/gl.h>
 #include <GLFW/glfw3.h>
 #include <glm/glm.hpp>
-
-//________________________________________________CALLBACK_FUNCTIONS_________________________________________________//
-
+#include <glm/gtc/matrix_transform.hpp>
+#include <glm/gtc/type_ptr.hpp>
+
+// Camera parameters for setting up the view matrix
+glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);  // Camera position
+glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f); // Camera direction
+glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f); // Camera up vector
+float yaw = -90.0f;  // Initial yaw angle
+float pitch = 0.0f;  // Initial pitch angle
+float lastX = 960.0f, lastY = 540.0f; // Last recorded mouse position
+bool firstMouse = true;  // Flag for initial mouse movement
+float fov = 45.0f;  // Field of view for the projection matrix
+float deltaTime = 0.0f;  // Time between current frame and last frame
+float lastFrame = 0.0f;  // Time of last frame
+
+// Movement speeds for camera movement
+float movementSpeed = 20.0f;
+float mouseSensitivity = 0.1f;
+
+// Global vector to store sphere indices for rendering
+std::vector<GLuint> sphereIndices;
+
+// Callback function to handle GLFW errors
 static void errorCallback(int error, const char* description) {
-	std::cerr << description;
+    std::cerr << description;
 }
 
+// Callback function to handle key input for camera movement and window closing
 static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
-	// close window when ESC has been pressed
-	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
-		glfwSetWindowShouldClose(window, GL_TRUE);
+    // Close window when ESC is pressed
+    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
+        glfwSetWindowShouldClose(window, GL_TRUE);
+
+    float cameraSpeed = movementSpeed * deltaTime;  // Adjust camera speed based on frame time
+
+    // Move camera in the direction it's facing
+    if (key == GLFW_KEY_W)
+        cameraPos += cameraSpeed * cameraFront;
+    if (key == GLFW_KEY_S)
+        cameraPos -= cameraSpeed * cameraFront;
+    // Strafe camera left and right
+    if (key == GLFW_KEY_A)
+        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
+    if (key == GLFW_KEY_D)
+        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
 }
 
+// Callback function for mouse button input (not used here but defined for completeness)
 static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {}
 
-static void mousePositionCallback(GLFWwindow* window, double xpos, double ypos) {}
-
-static void framebufferSizeCallback(GLFWwindow* window, int w, int h) { glViewport(0, 0, w, h); }
+// Callback function to handle mouse movement for adjusting camera orientation
+static void mousePositionCallback(GLFWwindow* window, double xpos, double ypos) {
+    if (firstMouse) {
+        lastX = xpos;  // Initialize lastX with the first mouse x position
+        lastY = ypos;  // Initialize lastY with the first mouse y position
+        firstMouse = false;  // Set firstMouse to false after initialization
+    }
+
+    float xoffset = xpos - lastX;  // Calculate horizontal offset
+    float yoffset = lastY - ypos;  // Calculate vertical offset (reversed since y-coordinates range from bottom to top)
+    lastX = xpos;  // Update lastX with current x position
+    lastY = ypos;  // Update lastY with current y position
+
+    xoffset *= mouseSensitivity;  // Adjust offset based on mouse sensitivity
+    yoffset *= mouseSensitivity;
+
+    yaw += xoffset;  // Update yaw angle
+    pitch += yoffset;  // Update pitch angle
+
+    // Constrain the pitch angle to avoid gimbal lock
+    if (pitch > 89.0f)
+        pitch = 89.0f;
+    if (pitch < -89.0f)
+        pitch = -89.0f;
+
+    // Calculate the new front vector based on updated yaw and pitch
+    glm::vec3 front;
+    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
+    front.y = sin(glm::radians(pitch));
+    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
+    cameraFront = glm::normalize(front);
+}
 
-//_________________________________________________INITIALIZATION____________________________________________________//
+// Callback function to handle window resizing
+static void framebufferSizeCallback(GLFWwindow* window, int w, int h) {
+    glViewport(0, 0, w, h);  // Update the OpenGL viewport to match the new window size
+}
 
+// Function to initialize GLFW and create a window with specified dimensions and title
 GLFWwindow* initialize(int width, int height, const std::string& title) {
-	GLFWwindow* window;
-	glfwSetErrorCallback(errorCallback);
-
-	// initialize glfw window
-	if (!glfwInit())
-		exit(EXIT_FAILURE);
-
-	// we want to use the opengl 4.6 core profile
-	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
-	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
-	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
-	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
-
-	// actually create the window
-	window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
-
-	// make sure the window creation was successful
-	if (!window) {
-		std::cerr << "Failed to open GLFW window.\n";
-		glfwTerminate();
-		exit(EXIT_FAILURE);
-	}
-	glfwMakeContextCurrent(window);
-
-	// initialize glad for loading all OpenGL functions
-	if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
-		std::cerr << "Something went wrong!\n";
-		exit(EXIT_FAILURE);
-	}
-
-	// print some information about the supported OpenGL version
-	const GLubyte* renderer = glGetString(GL_RENDERER);
-	const GLubyte* version = glGetString(GL_VERSION);
-	std::cout << "Renderer: " << renderer << '\n';
-	std::cout << "OpenGL version supported " << version << '\n';
-
-	// register user callbacks
-	glfwSetKeyCallback(window, keyCallback);
-	glfwSetMouseButtonCallback(window, mouseButtonCallback);
-	glfwSetCursorPosCallback(window, mousePositionCallback);
-	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
-
-	// set the clear color of the window
-	glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
-
-	return window;
+    GLFWwindow* window;
+    glfwSetErrorCallback(errorCallback);  // Set the error callback function
+
+    // Initialize GLFW library
+    if (!glfwInit())
+        exit(EXIT_FAILURE);
+
+    // Set GLFW window hints for OpenGL version and profile
+    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
+    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
+    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
+    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
+
+    // Create a windowed mode window and its OpenGL context
+    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
+    if (!window) {
+        std::cerr << "Failed to open GLFW window.\n";
+        glfwTerminate();
+        exit(EXIT_FAILURE);
+    }
+    glfwMakeContextCurrent(window);  // Make the window's context current
+
+    // Load all OpenGL function pointers using GLAD
+    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
+        std::cerr << "Failed to initialize GLAD.\n";
+        exit(EXIT_FAILURE);
+    }
+
+    // Print the renderer and OpenGL version
+    const GLubyte* renderer = glGetString(GL_RENDERER);
+    const GLubyte* version = glGetString(GL_VERSION);
+    std::cout << "Renderer: " << renderer << '\n';
+    std::cout << "OpenGL version supported: " << version << '\n';
+
+    // Set GLFW callback functions for input handling
+    glfwSetKeyCallback(window, keyCallback);
+    glfwSetMouseButtonCallback(window, mouseButtonCallback);
+    glfwSetCursorPosCallback(window, mousePositionCallback);
+    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
+
+    // Set the clear color for the window
+    glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
+
+    // Enable depth testing for correct rendering of 3D objects
+    glEnable(GL_DEPTH_TEST);
+
+    // Hide the mouse cursor and capture it within the window
+    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
+
+    return window;
+}
+
+// Struct to hold vertex data (position and color)
+struct Vertex {
+    glm::vec3 pos;
+    glm::vec3 color;
+};
+
+// Function to generate vertices for a sphere
+std::vector<Vertex> generateSphere(float radius, int sectorCount, int stackCount) {
+    std::vector<Vertex> vertices;
+    float x, y, z, xy;  // Variables for vertex positions
+    float sectorStep = 2 * M_PI / sectorCount;  // Step size for sectors (longitude)
+    float stackStep = M_PI / stackCount;  // Step size for stacks (latitude)
+    float sectorAngle, stackAngle;
+
+    // Loop through stacks and sectors to generate vertices
+    for (int i = 0; i <= stackCount; ++i) {
+        stackAngle = M_PI / 2 - i * stackStep;  // Calculate current stack angle
+        xy = radius * cosf(stackAngle);  // Calculate x and y based on stack angle
+        z = radius * sinf(stackAngle);  // Calculate z based on stack angle
+
+        for (int j = 0; j <= sectorCount; ++j) {
+            sectorAngle = j * sectorStep;  // Calculate current sector angle
+
+            x = xy * cosf(sectorAngle);  // Calculate x position
+            y = xy * sinf(sectorAngle);  // Calculate y position
+            vertices.push_back({ glm::vec3(x, y, z), glm::vec3((x + radius) / (2 * radius), (y + radius) / (2 * radius), (z + radius) / (2 * radius)) });
+        }
+    }
+
+    return vertices;
 }
 
-GLuint createBuffers() {
-	// specify the layout of the vertex data, being the vertex position followed by the vertex color
-	struct Vertex {
-		glm::vec3 pos;
-		glm::vec3 color;
-	};
-
-	// we specify a triangle with red, green, blue at the tips of the triangle
-	Vertex vertexData[] = {
-		Vertex{glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(1.f, 0.f, 0.f)},
-		Vertex{glm::vec3( 0.5f, -0.5f, 0.0f), glm::vec3(0.f, 1.f, 0.f)},
-		Vertex{glm::vec3( 0.0f,  0.5f, 0.0f), glm::vec3(0.f, 0.f, 1.f)}
-	};
-
-	// create the vertex array object that holds all vertex buffers
-	GLuint vao;
-	glGenVertexArrays(1, &vao);
-	glBindVertexArray(vao);
-
-	// create a vertex buffer that contains all vertex positions and copy the vertex positions into that buffer
-	GLuint vbo;
-	glGenBuffers(1, &vbo);
-	glBindBuffer(GL_ARRAY_BUFFER, vbo);
-	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);
-
-	// we need to tell the buffer in which format the data is and we need to explicitly enable it
-	// first we specify the layout of the vertex position
-	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(struct Vertex, pos)));
-	glEnableVertexAttribArray(0);
-	// then we specify the layout of the vertex color
-	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(struct Vertex, color)));
-	glEnableVertexAttribArray(1);
-
-	return vao;
+// Function to generate indices for drawing sphere triangles
+std::vector<GLuint> generateIndices(int sectorCount, int stackCount) {
+    std::vector<GLuint> indices;
+    int k1, k2;  // Indices for stacks and sectors
+
+    // Loop through stacks and sectors to generate indices
+    for (int i = 0; i < stackCount; ++i) {
+        k1 = i * (sectorCount + 1);  // Index for current stack
+        k2 = k1 + sectorCount + 1;  // Index for next stack
+
+        for (int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
+            if (i != 0) {
+                indices.push_back(k1);  // First triangle
+                indices.push_back(k2);
+                indices.push_back(k1 + 1);
+            }
+
+            if (i != (stackCount - 1)) {
+                indices.push_back(k1 + 1);  // Second triangle
+                indices.push_back(k2);
+                indices.push_back(k2 + 1);
+            }
+        }
+    }
+
+    return indices;
 }
 
+// Function to create buffers for the sphere
+GLuint createBuffersForSphere(float radius, int sectorCount, int stackCount) {
+    std::vector<Vertex> vertices = generateSphere(radius, sectorCount, stackCount);  // Generate sphere vertices
+    sphereIndices = generateIndices(sectorCount, stackCount);  // Generate sphere indices
+
+    GLuint vao, vbo, ebo;
+    glGenVertexArrays(1, &vao);  // Create a vertex array object
+    glGenBuffers(1, &vbo);  // Create a vertex buffer object
+    glGenBuffers(1, &ebo);  // Create an element buffer object
+
+    glBindVertexArray(vao);  // Bind the vertex array object
+
+    glBindBuffer(GL_ARRAY_BUFFER, vbo);  // Bind the vertex buffer object
+    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);  // Copy vertex data to buffer
+
+    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);  // Bind the element buffer object
+    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(GLuint), &sphereIndices[0], GL_STATIC_DRAW);  // Copy index data to buffer
+
+    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, pos)));  // Specify vertex position attribute
+    glEnableVertexAttribArray(0);  // Enable the vertex position attribute
+    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, color)));  // Specify vertex color attribute
+    glEnableVertexAttribArray(1);  // Enable the vertex color attribute
+
+    glBindVertexArray(0);  // Unbind the vertex array object
+
+    return vao;
+}
+
+// Function to compile a shader from a file
 GLuint compileShader(const std::string& path, GLenum shaderType) {
-	// grab the contents of the file and store the source code in a string
-	std::ifstream filestream(path);
-	std::string shaderSource((std::istreambuf_iterator<char>(filestream)),
-		std::istreambuf_iterator<char>());
-
-	// create and compile the shader
-	GLuint shaderHandle = glCreateShader(shaderType);
-	const char* shaderSourcePtr = shaderSource.c_str();
-	glShaderSource(shaderHandle, 1, &shaderSourcePtr, nullptr);
-	glCompileShader(shaderHandle);
-
-	// check if compilation was successful
-	int  success;
-	char infoLog[512];
-	glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &success);
-	if (!success) {
-		glGetShaderInfoLog(shaderHandle, 512, nullptr, infoLog);
-		std::cerr << "Error while compiling shader\n" << infoLog << std::endl;
-	}
-
-	// return the shader handle
-	return shaderHandle;
+    std::ifstream filestream(path);  // Open the shader file
+    std::string shaderSource((std::istreambuf_iterator<char>(filestream)), std::istreambuf_iterator<char>());  // Read the shader source
+
+    GLuint shaderHandle = glCreateShader(shaderType);  // Create a shader object
+    const char* shaderSourcePtr = shaderSource.c_str();  // Get a pointer to the shader source
+    glShaderSource(shaderHandle, 1, &shaderSourcePtr, nullptr);  // Set the shader source
+    glCompileShader(shaderHandle);  // Compile the shader
+
+    int success;
+    char infoLog[512];
+    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &success);  // Check for compilation errors
+    if (!success) {
+        glGetShaderInfoLog(shaderHandle, 512, nullptr, infoLog);  // Get the error log
+        std::cerr << "Error while compiling shader\n" << infoLog << std::endl;
+    }
+
+    return shaderHandle;
 }
 
+// Function to create a shader program from vertex and fragment shaders
 GLuint createShaderProgram(const std::string& vertexShaderPath, const std::string& fragmentShaderPath) {
-	// create and compile shaders
-	GLenum vertexShader = compileShader(vertexShaderPath, GL_VERTEX_SHADER);
-	GLenum fragmentShader = compileShader(fragmentShaderPath, GL_FRAGMENT_SHADER);
-
-	// create a shader program, attach both shaders and link them together
-	GLuint shaderProgram = glCreateProgram();
-	glAttachShader(shaderProgram, vertexShader);
-	glAttachShader(shaderProgram, fragmentShader);
-	glLinkProgram(shaderProgram);
-
-	// check for errors while linking the shaders together
-	int  success;
-	char infoLog[512];
-	glGetShaderiv(shaderProgram, GL_COMPILE_STATUS, &success);
-	if (!success) {
-		glGetShaderInfoLog(shaderProgram, 512, nullptr, infoLog);
-		std::cerr << "Error while linking shaders\n" << infoLog << std::endl;
-	}
-
-	// after creating the shader program we don't need the two shaders anymore
-	glDeleteShader(vertexShader);
-	glDeleteShader(fragmentShader);
-
-	// return the shader program handle
-	return shaderProgram;
+    GLuint vertexShader = compileShader(vertexShaderPath, GL_VERTEX_SHADER);  // Compile the vertex shader
+    GLuint fragmentShader = compileShader(fragmentShaderPath, GL_FRAGMENT_SHADER);  // Compile the fragment shader
+
+    GLuint shaderProgram = glCreateProgram();  // Create a shader program
+    glAttachShader(shaderProgram, vertexShader);  // Attach the vertex shader
+    glAttachShader(shaderProgram, fragmentShader);  // Attach the fragment shader
+    glLinkProgram(shaderProgram);  // Link the shaders into the program
+
+    int success;
+    char infoLog[512];
+    glGetShaderiv(shaderProgram, GL_LINK_STATUS, &success);  // Check for linking errors
+    if (!success) {
+        glGetShaderInfoLog(shaderProgram, 512, nullptr, infoLog);  // Get the error log
+        std::cerr << "Error while linking shaders\n" << infoLog << std::endl;
+    }
+
+    glDeleteShader(vertexShader);  // Delete the vertex shader as it's no longer needed
+    glDeleteShader(fragmentShader);  // Delete the fragment shader as it's no longer needed
+
+    return shaderProgram;
 }
 
-//______________________________________________________RENDER_______________________________________________________//
+// Function to render the spheres
+void render(GLuint shaderProgram, GLuint vao, const std::vector<glm::vec3>& spherePositions) {
+    glUseProgram(shaderProgram);  // Use the shader program
 
-void render(GLuint shaderProgram, GLuint vao) {
-	glUseProgram(shaderProgram);
-	glBindVertexArray(vao);
-	glDrawArrays(GL_TRIANGLES, 0, 3);
-}
+    // Set up the view matrix based on the camera's position, direction, and up vector
+    glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
+    // Set up the projection matrix with perspective projection
+    glm::mat4 projection = glm::perspective(glm::radians(fov), 1920.0f / 1080.0f, 0.1f, 100.0f);
 
-//______________________________________________________CLEANUP______________________________________________________//
+    // Pass the view matrix to the shader
+    GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
+    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
 
-void cleanup(GLFWwindow* window, GLuint& shaderProgram, GLuint& vao) {
-	// do some custom cleanup here
-	glDeleteProgram(shaderProgram);
-	glDeleteVertexArrays(1, &vao);
+    // Pass the projection matrix to the shader
+    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
+    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
+
+    glBindVertexArray(vao);  // Bind the vertex array object
 
-	// lastly destroy the window and terminate glfw
-	glfwDestroyWindow(window);
-	glfwTerminate();
+    // Loop through each sphere position and render the sphere
+    for (const auto& position : spherePositions) {
+        glm::mat4 model = glm::mat4(1.0f);  // Create the model matrix
+        GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
+        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));  // Pass the model matrix to the shader
+
+        GLint spherePosLoc = glGetUniformLocation(shaderProgram, "spherePosition");
+        glUniform3fv(spherePosLoc, 1, glm::value_ptr(position));  // Pass the sphere position to the shader
+
+        glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0);  // Draw the sphere using the index buffer
+    }
 }
 
-//_______________________________________________________MAIN________________________________________________________//
+// Function to clean up resources
+void cleanup(GLFWwindow* window, GLuint& shaderProgram, GLuint& vao) {
+    glDeleteProgram(shaderProgram);  // Delete the shader program
+    glDeleteVertexArrays(1, &vao);  // Delete the vertex array object
+
+    glfwDestroyWindow(window);  // Destroy the GLFW window
+    glfwTerminate();  // Terminate GLFW
+}
 
+// Main function
 int main() {
-	// create a window with the specified width, height and title and initialize OpenGL
-	GLFWwindow* window = initialize(640, 480, "OpenGL Starter Project");
-	GLuint shaderProgram = createShaderProgram(
-		ASSETS_PATH"/shaders/test.vert.glsl",
-		ASSETS_PATH"/shaders/test.frag.glsl");
-	GLuint vao = createBuffers();
-
-	// loop until the user presses ESC or the window is closed programmatically
-	while (!glfwWindowShouldClose(window)) {
-		// clear the back buffer with the specified color and the depth buffer with 1
-		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
-
-		// render to back buffer
-		render(shaderProgram, vao);
-
-		// switch front and back buffers
-		glfwSwapBuffers(window);
-		glfwPollEvents();
-	}
-
-	// clean up all created objects
-	cleanup(window, shaderProgram, vao);
-
-	// program exits properly
-	exit(EXIT_SUCCESS);
-}
\ No newline at end of file
+    // Initialize GLFW and create a window
+    GLFWwindow* window = initialize(1920, 1080, "OpenGL Sphere Drawing");
+
+    // Create the shader program
+    GLuint shaderProgram = createShaderProgram(
+        ASSETS_PATH"/shaders/test.vert.glsl",
+        ASSETS_PATH"/shaders/test.frag.glsl");
+
+    // Define parameters for the sphere
+    float radius = 1.0f;
+    int sectorCount = 36;
+    int stackCount = 18;
+
+    // Create buffers for the sphere
+    GLuint vao = createBuffersForSphere(radius, sectorCount, stackCount);
+
+    // Define positions for multiple spheres
+    std::vector<glm::vec3> spherePositions = {
+        {0.0f, 0.0f, 0.0f},
+        {2.0f, 2.0f, 0.0f},
+        {-2.0f, -2.0f, 0.0f},
+        {2.0f, -2.0f, 0.0f},
+        {-2.0f, 2.0f, 0.0f}
+    };
+
+    // Main render loop
+    while (!glfwWindowShouldClose(window)) {
+        // Calculate delta time
+        float currentFrame = glfwGetTime();
+        deltaTime = currentFrame - lastFrame;
+        lastFrame = currentFrame;
+
+        // Clear the color and depth buffers
+        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
+
+        // Render the spheres
+        render(shaderProgram, vao, spherePositions);
+
+        // Swap the front and back buffers
+        glfwSwapBuffers(window);
+        glfwPollEvents();  // Poll for and process events
+    }
+
+    // Clean up resources
+    cleanup(window, shaderProgram, vao);
+
+    // Program exits successfully
+    exit(EXIT_SUCCESS);
+}
```

And I also changed the shader codes to support the movement of the camera (The actual matrix multiplication is here):

```diff
diff --git a/assets/shaders/test.frag.glsl b/assets/shaders/test.frag.glsl
index 077ca27..2f5240b 100644
--- a/assets/shaders/test.frag.glsl
+++ b/assets/shaders/test.frag.glsl
@@ -1,9 +1,8 @@
-#version 330 core
+#version 460 core
 
-layout(location = 0) out vec3 outColor;
-
-in vec3 oColor;
+in vec3 ourColor;                          // Input color from vertex shader
+out vec4 FragColor;                        // Output color of the fragment
 
 void main() {
-    outColor = oColor;
-}
\ No newline at end of file
+    FragColor = vec4(ourColor, 1.0f);      // Set the fragment color to the input color with full opacity
+}
diff --git a/assets/shaders/test.vert.glsl b/assets/shaders/test.vert.glsl
index efe8d5b..72e9689 100644
--- a/assets/shaders/test.vert.glsl
+++ b/assets/shaders/test.vert.glsl
@@ -1,11 +1,17 @@
-#version 330 core
+#version 460 core
 
-layout (location = 0) in vec3 aPos;
-layout (location = 1) in vec3 aColor;
+layout(location = 0) in vec3 aPos;          // Vertex position attribute
+layout(location = 1) in vec3 aColor;        // Vertex color attribute
 
-out vec3 oColor;
+out vec3 ourColor;                         // Output color to fragment shader
+
+uniform mat4 model;                        // Model matrix to transform vertices to world space
+uniform mat4 view;                         // View matrix to transform vertices to camera space
+uniform mat4 projection;                   // Projection matrix to transform vertices to clip space
+uniform vec3 spherePosition;               // Position of the sphere in world space
 
 void main() {
-    gl_Position = vec4(aPos.x, aPos.y, aPos.z, 1.0);
-    oColor = aColor;
-}
\ No newline at end of file
+    // Calculate the final position of the vertex by transforming it through model, view, and projection matrices
+    gl_Position = projection * view * model * vec4(aPos + spherePosition, 1.0); 
+    ourColor = aColor;                     // Pass the vertex color to the fragment shader
+}
```

Then, I copy over the A3 naive particle physics engine into `pbd.h`, and used the OpenGL to draw the particles as the spheres we did above, so we have a very basic visualization engine that utilizes GPU so it's very fast:

![image](/screenshots/sc3.png)

```diff
diff --git a/src/main.cpp b/src/main.cpp
index 69be4f2..89ccba3 100644
--- a/src/main.cpp
+++ b/src/main.cpp
@@ -10,367 +10,394 @@
 #include <glm/gtc/matrix_transform.hpp>
 #include <glm/gtc/type_ptr.hpp>
 
+#include "pbd.h"
+
 // Camera parameters for setting up the view matrix
-glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);  // Camera position
+glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);	  // Camera position
 glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f); // Camera direction
-glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f); // Camera up vector
-float yaw = -90.0f;  // Initial yaw angle
-float pitch = 0.0f;  // Initial pitch angle
-float lastX = 960.0f, lastY = 540.0f; // Last recorded mouse position
-bool firstMouse = true;  // Flag for initial mouse movement
-float fov = 45.0f;  // Field of view for the projection matrix
-float deltaTime = 0.0f;  // Time between current frame and last frame
-float lastFrame = 0.0f;  // Time of last frame
+glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);	  // Camera up vector
+float yaw = -90.0f;									  // Initial yaw angle
+float pitch = 0.0f;									  // Initial pitch angle
+float lastX = 960.0f, lastY = 540.0f;				  // Last recorded mouse position
+bool firstMouse = true;								  // Flag for initial mouse movement
+float fov = 45.0f;									  // Field of view for the projection matrix
+float deltaTime = 0.0f;								  // Time between current frame and last frame
+float lastFrame = 0.0f;								  // Time of last frame
 
 // Movement speeds for camera movement
-float movementSpeed = 30.0f;
+float movementSpeed = 10.0f;
 float mouseSensitivity = 0.1f;
 
+// Define parameters for the sphere
+int sectorCount = 36;
+int stackCount = 18;
+
 // Global vector to store sphere indices for rendering
 std::vector<GLuint> sphereIndices;
 
 // Callback function to handle GLFW errors
-static void errorCallback(int error, const char* description) {
-    std::cerr << description;
+static void errorCallback(int error, const char *description)
+{
+	std::cerr << description;
 }
 
 // Callback function to handle key input for camera movement and window closing
-static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
-    // Close window when ESC is pressed
-    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
-        glfwSetWindowShouldClose(window, GL_TRUE);
-
-    float cameraSpeed = movementSpeed * deltaTime;  // Adjust camera speed based on frame time
-
-    // Move camera in the direction it's facing
-    if (key == GLFW_KEY_W)
-        cameraPos += cameraSpeed * cameraFront;
-    if (key == GLFW_KEY_S)
-        cameraPos -= cameraSpeed * cameraFront;
-    // Strafe camera left and right
-    if (key == GLFW_KEY_A)
-        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
-    if (key == GLFW_KEY_D)
-        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
+static void keyCallback(GLFWwindow *window, int key, int scancode, int action, int mods)
+{
+	// Close window when ESC is pressed
+	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
+		glfwSetWindowShouldClose(window, GL_TRUE);
+
+	float cameraSpeed = movementSpeed * deltaTime; // Adjust camera speed based on frame time
+
+	// Move camera in the direction it's facing
+	if (key == GLFW_KEY_W)
+		cameraPos += cameraSpeed * cameraFront;
+	if (key == GLFW_KEY_S)
+		cameraPos -= cameraSpeed * cameraFront;
+	// Strafe camera left and right
+	if (key == GLFW_KEY_A)
+		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
+	if (key == GLFW_KEY_D)
+		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
 }
 
 // Callback function for mouse button input (not used here but defined for completeness)
-static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {}
+static void mouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {}
 
 // Callback function to handle mouse movement for adjusting camera orientation
-static void mousePositionCallback(GLFWwindow* window, double xpos, double ypos) {
-    if (firstMouse) {
-        lastX = xpos;  // Initialize lastX with the first mouse x position
-        lastY = ypos;  // Initialize lastY with the first mouse y position
-        firstMouse = false;  // Set firstMouse to false after initialization
-    }
-
-    float xoffset = xpos - lastX;  // Calculate horizontal offset
-    float yoffset = lastY - ypos;  // Calculate vertical offset (reversed since y-coordinates range from bottom to top)
-    lastX = xpos;  // Update lastX with current x position
-    lastY = ypos;  // Update lastY with current y position
-
-    xoffset *= mouseSensitivity;  // Adjust offset based on mouse sensitivity
-    yoffset *= mouseSensitivity;
-
-    yaw += xoffset;  // Update yaw angle
-    pitch += yoffset;  // Update pitch angle
-
-    // Constrain the pitch angle to avoid gimbal lock
-    if (pitch > 89.0f)
-        pitch = 89.0f;
-    if (pitch < -89.0f)
-        pitch = -89.0f;
-
-    // Calculate the new front vector based on updated yaw and pitch
-    glm::vec3 front;
-    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
-    front.y = sin(glm::radians(pitch));
-    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
-    cameraFront = glm::normalize(front);
+static void mousePositionCallback(GLFWwindow *window, double xpos, double ypos)
+{
+	if (firstMouse)
+	{
+		lastX = xpos;		// Initialize lastX with the first mouse x position
+		lastY = ypos;		// Initialize lastY with the first mouse y position
+		firstMouse = false; // Set firstMouse to false after initialization
+	}
+
+	float xoffset = xpos - lastX; // Calculate horizontal offset
+	float yoffset = lastY - ypos; // Calculate vertical offset (reversed since y-coordinates range from bottom to top)
+	lastX = xpos;				  // Update lastX with current x position
+	lastY = ypos;				  // Update lastY with current y position
+
+	xoffset *= mouseSensitivity; // Adjust offset based on mouse sensitivity
+	yoffset *= mouseSensitivity;
+
+	yaw += xoffset;	  // Update yaw angle
+	pitch += yoffset; // Update pitch angle
+
+	// Constrain the pitch angle to avoid gimbal lock
+	if (pitch > 89.0f)
+		pitch = 89.0f;
+	if (pitch < -89.0f)
+		pitch = -89.0f;
+
+	// Calculate the new front vector based on updated yaw and pitch
+	glm::vec3 front;
+	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
+	front.y = sin(glm::radians(pitch));
+	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
+	cameraFront = glm::normalize(front);
 }
 
 // Callback function to handle window resizing
-static void framebufferSizeCallback(GLFWwindow* window, int w, int h) {
-    glViewport(0, 0, w, h);  // Update the OpenGL viewport to match the new window size
+static void framebufferSizeCallback(GLFWwindow *window, int w, int h)
+{
+	glViewport(0, 0, w, h); // Update the OpenGL viewport to match the new window size
 }
 
 // Function to initialize GLFW and create a window with specified dimensions and title
-GLFWwindow* initialize(int width, int height, const std::string& title) {
-    GLFWwindow* window;
-    glfwSetErrorCallback(errorCallback);  // Set the error callback function
-
-    // Initialize GLFW library
-    if (!glfwInit())
-        exit(EXIT_FAILURE);
-
-    // Set GLFW window hints for OpenGL version and profile
-    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
-    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
-    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
-    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
-
-    // Create a windowed mode window and its OpenGL context
-    window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
-    if (!window) {
-        std::cerr << "Failed to open GLFW window.\n";
-        glfwTerminate();
-        exit(EXIT_FAILURE);
-    }
-    glfwMakeContextCurrent(window);  // Make the window's context current
-
-    // Load all OpenGL function pointers using GLAD
-    if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
-        std::cerr << "Failed to initialize GLAD.\n";
-        exit(EXIT_FAILURE);
-    }
-
-    // Print the renderer and OpenGL version
-    const GLubyte* renderer = glGetString(GL_RENDERER);
-    const GLubyte* version = glGetString(GL_VERSION);
-    std::cout << "Renderer: " << renderer << '\n';
-    std::cout << "OpenGL version supported: " << version << '\n';
-
-    // Set GLFW callback functions for input handling
-    glfwSetKeyCallback(window, keyCallback);
-    glfwSetMouseButtonCallback(window, mouseButtonCallback);
-    glfwSetCursorPosCallback(window, mousePositionCallback);
-    glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
-
-    // Set the clear color for the window
-    glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
-
-    // Enable depth testing for correct rendering of 3D objects
-    glEnable(GL_DEPTH_TEST);
-
-    // Hide the mouse cursor and capture it within the window
-    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
-
-    return window;
+GLFWwindow *initialize(int width, int height, const std::string &title)
+{
+	GLFWwindow *window;
+	glfwSetErrorCallback(errorCallback); // Set the error callback function
+
+	// Initialize GLFW library
+	if (!glfwInit())
+		exit(EXIT_FAILURE);
+
+	// Set GLFW window hints for OpenGL version and profile
+	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
+	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
+	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
+	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
+
+	// Create a windowed mode window and its OpenGL context
+	window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);
+	if (!window)
+	{
+		std::cerr << "Failed to open GLFW window.\n";
+		glfwTerminate();
+		exit(EXIT_FAILURE);
+	}
+	glfwMakeContextCurrent(window); // Make the window's context current
+
+	// Load all OpenGL function pointers using GLAD
+	if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress))
+	{
+		std::cerr << "Failed to initialize GLAD.\n";
+		exit(EXIT_FAILURE);
+	}
+
+	// Print the renderer and OpenGL version
+	const GLubyte *renderer = glGetString(GL_RENDERER);
+	const GLubyte *version = glGetString(GL_VERSION);
+	std::cout << "Renderer: " << renderer << '\n';
+	std::cout << "OpenGL version supported: " << version << '\n';
+
+	// Set GLFW callback functions for input handling
+	glfwSetKeyCallback(window, keyCallback);
+	glfwSetMouseButtonCallback(window, mouseButtonCallback);
+	glfwSetCursorPosCallback(window, mousePositionCallback);
+	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
+
+	// Set the clear color for the window
+	glClearColor(0.3f, 0.3f, 0.3f, 0.0f);
+
+	// Enable depth testing for correct rendering of 3D objects
+	glEnable(GL_DEPTH_TEST);
+
+	// Hide the mouse cursor and capture it within the window
+	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
+
+	return window;
 }
 
 // Struct to hold vertex data (position and color)
-struct Vertex {
-    glm::vec3 pos;
-    glm::vec3 color;
+struct Vertex
+{
+	glm::vec3 pos;
+	glm::vec3 color;
 };
 
 // Function to generate vertices for a sphere
-std::vector<Vertex> generateSphere(float radius, int sectorCount, int stackCount) {
-    std::vector<Vertex> vertices;
-    float x, y, z, xy;  // Variables for vertex positions
-    float sectorStep = 2 * M_PI / sectorCount;  // Step size for sectors (longitude)
-    float stackStep = M_PI / stackCount;  // Step size for stacks (latitude)
-    float sectorAngle, stackAngle;
-
-    // Loop through stacks and sectors to generate vertices
-    for (int i = 0; i <= stackCount; ++i) {
-        stackAngle = M_PI / 2 - i * stackStep;  // Calculate current stack angle
-        xy = radius * cosf(stackAngle);  // Calculate x and y based on stack angle
-        z = radius * sinf(stackAngle);  // Calculate z based on stack angle
-
-        for (int j = 0; j <= sectorCount; ++j) {
-            sectorAngle = j * sectorStep;  // Calculate current sector angle
-
-            x = xy * cosf(sectorAngle);  // Calculate x position
-            y = xy * sinf(sectorAngle);  // Calculate y position
-            vertices.push_back({ glm::vec3(x, y, z), glm::vec3((x + radius) / (2 * radius), (y + radius) / (2 * radius), (z + radius) / (2 * radius)) });
-        }
-    }
-
-    return vertices;
+std::vector<Vertex> generateSphere(float radius, int sectorCount, int stackCount)
+{
+	std::vector<Vertex> vertices;
+	float x, y, z, xy;						   // Variables for vertex positions
+	float sectorStep = 2 * M_PI / sectorCount; // Step size for sectors (longitude)
+	float stackStep = M_PI / stackCount;	   // Step size for stacks (latitude)
+	float sectorAngle, stackAngle;
+
+	// Loop through stacks and sectors to generate vertices
+	for (int i = 0; i <= stackCount; ++i)
+	{
+		stackAngle = M_PI / 2 - i * stackStep; // Calculate current stack angle
+		xy = radius * cosf(stackAngle);		   // Calculate x and y based on stack angle
+		z = radius * sinf(stackAngle);		   // Calculate z based on stack angle
+
+		for (int j = 0; j <= sectorCount; ++j)
+		{
+			sectorAngle = j * sectorStep; // Calculate current sector angle
+
+			x = xy * cosf(sectorAngle); // Calculate x position
+			y = xy * sinf(sectorAngle); // Calculate y position
+			vertices.push_back({glm::vec3(x, y, z), glm::vec3((x + radius) / (2 * radius), (y + radius) / (2 * radius), (z + radius) / (2 * radius))});
+		}
+	}
+
+	return vertices;
 }
 
 // Function to generate indices for drawing sphere triangles
-std::vector<GLuint> generateIndices(int sectorCount, int stackCount) {
-    std::vector<GLuint> indices;
-    int k1, k2;  // Indices for stacks and sectors
-
-    // Loop through stacks and sectors to generate indices
-    for (int i = 0; i < stackCount; ++i) {
-        k1 = i * (sectorCount + 1);  // Index for current stack
-        k2 = k1 + sectorCount + 1;  // Index for next stack
-
-        for (int j = 0; j < sectorCount; ++j, ++k1, ++k2) {
-            if (i != 0) {
-                indices.push_back(k1);  // First triangle
-                indices.push_back(k2);
-                indices.push_back(k1 + 1);
-            }
-
-            if (i != (stackCount - 1)) {
-                indices.push_back(k1 + 1);  // Second triangle
-                indices.push_back(k2);
-                indices.push_back(k2 + 1);
-            }
-        }
-    }
-
-    return indices;
+std::vector<GLuint> generateIndices(int sectorCount, int stackCount)
+{
+	std::vector<GLuint> indices;
+	int k1, k2; // Indices for stacks and sectors
+
+	// Loop through stacks and sectors to generate indices
+	for (int i = 0; i < stackCount; ++i)
+	{
+		k1 = i * (sectorCount + 1); // Index for current stack
+		k2 = k1 + sectorCount + 1;	// Index for next stack
+
+		for (int j = 0; j < sectorCount; ++j, ++k1, ++k2)
+		{
+			if (i != 0)
+			{
+				indices.push_back(k1); // First triangle
+				indices.push_back(k2);
+				indices.push_back(k1 + 1);
+			}
+
+			if (i != (stackCount - 1))
+			{
+				indices.push_back(k1 + 1); // Second triangle
+				indices.push_back(k2);
+				indices.push_back(k2 + 1);
+			}
+		}
+	}
+
+	return indices;
 }
 
 // Function to create buffers for the sphere
-GLuint createBuffersForSphere(float radius, int sectorCount, int stackCount) {
-    std::vector<Vertex> vertices = generateSphere(radius, sectorCount, stackCount);  // Generate sphere vertices
-    sphereIndices = generateIndices(sectorCount, stackCount);  // Generate sphere indices
+GLuint createBuffersForSphere(float radius, int sectorCount, int stackCount)
+{
+	std::vector<Vertex> vertices = generateSphere(radius, sectorCount, stackCount); // Generate sphere vertices
+	sphereIndices = generateIndices(sectorCount, stackCount);						// Generate sphere indices
 
-    GLuint vao, vbo, ebo;
-    glGenVertexArrays(1, &vao);  // Create a vertex array object
-    glGenBuffers(1, &vbo);  // Create a vertex buffer object
-    glGenBuffers(1, &ebo);  // Create an element buffer object
+	GLuint vao, vbo, ebo;
+	glGenVertexArrays(1, &vao); // Create a vertex array object
+	glGenBuffers(1, &vbo);		// Create a vertex buffer object
+	glGenBuffers(1, &ebo);		// Create an element buffer object
 
-    glBindVertexArray(vao);  // Bind the vertex array object
+	glBindVertexArray(vao); // Bind the vertex array object
 
-    glBindBuffer(GL_ARRAY_BUFFER, vbo);  // Bind the vertex buffer object
-    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW);  // Copy vertex data to buffer
+	glBindBuffer(GL_ARRAY_BUFFER, vbo);															   // Bind the vertex buffer object
+	glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), &vertices[0], GL_STATIC_DRAW); // Copy vertex data to buffer
 
-    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);  // Bind the element buffer object
-    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(GLuint), &sphereIndices[0], GL_STATIC_DRAW);  // Copy index data to buffer
+	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);																		 // Bind the element buffer object
+	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndices.size() * sizeof(GLuint), &sphereIndices[0], GL_STATIC_DRAW); // Copy index data to buffer
 
-    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, pos)));  // Specify vertex position attribute
-    glEnableVertexAttribArray(0);  // Enable the vertex position attribute
-    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(Vertex, color)));  // Specify vertex color attribute
-    glEnableVertexAttribArray(1);  // Enable the vertex color attribute
+	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)(offsetof(Vertex, pos)));	// Specify vertex position attribute
+	glEnableVertexAttribArray(0);																		// Enable the vertex position attribute
+	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)(offsetof(Vertex, color))); // Specify vertex color attribute
+	glEnableVertexAttribArray(1);																		// Enable the vertex color attribute
 
-    glBindVertexArray(0);  // Unbind the vertex array object
+	glBindVertexArray(0); // Unbind the vertex array object
 
-    return vao;
+	return vao;
 }
 
 // Function to compile a shader from a file
-GLuint compileShader(const std::string& path, GLenum shaderType) {
-    std::ifstream filestream(path);  // Open the shader file
-    std::string shaderSource((std::istreambuf_iterator<char>(filestream)), std::istreambuf_iterator<char>());  // Read the shader source
-
-    GLuint shaderHandle = glCreateShader(shaderType);  // Create a shader object
-    const char* shaderSourcePtr = shaderSource.c_str();  // Get a pointer to the shader source
-    glShaderSource(shaderHandle, 1, &shaderSourcePtr, nullptr);  // Set the shader source
-    glCompileShader(shaderHandle);  // Compile the shader
-
-    int success;
-    char infoLog[512];
-    glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &success);  // Check for compilation errors
-    if (!success) {
-        glGetShaderInfoLog(shaderHandle, 512, nullptr, infoLog);  // Get the error log
-        std::cerr << "Error while compiling shader\n" << infoLog << std::endl;
-    }
-
-    return shaderHandle;
+GLuint compileShader(const std::string &path, GLenum shaderType)
+{
+	std::ifstream filestream(path);																			  // Open the shader file
+	std::string shaderSource((std::istreambuf_iterator<char>(filestream)), std::istreambuf_iterator<char>()); // Read the shader source
+
+	GLuint shaderHandle = glCreateShader(shaderType);			// Create a shader object
+	const char *shaderSourcePtr = shaderSource.c_str();			// Get a pointer to the shader source
+	glShaderSource(shaderHandle, 1, &shaderSourcePtr, nullptr); // Set the shader source
+	glCompileShader(shaderHandle);								// Compile the shader
+
+	int success;
+	char infoLog[512];
+	glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &success); // Check for compilation errors
+	if (!success)
+	{
+		glGetShaderInfoLog(shaderHandle, 512, nullptr, infoLog); // Get the error log
+		std::cerr << "Error while compiling shader\n"
+				  << infoLog << std::endl;
+	}
+
+	return shaderHandle;
 }
 
 // Function to create a shader program from vertex and fragment shaders
-GLuint createShaderProgram(const std::string& vertexShaderPath, const std::string& fragmentShaderPath) {
-    GLuint vertexShader = compileShader(vertexShaderPath, GL_VERTEX_SHADER);  // Compile the vertex shader
-    GLuint fragmentShader = compileShader(fragmentShaderPath, GL_FRAGMENT_SHADER);  // Compile the fragment shader
-
-    GLuint shaderProgram = glCreateProgram();  // Create a shader program
-    glAttachShader(shaderProgram, vertexShader);  // Attach the vertex shader
-    glAttachShader(shaderProgram, fragmentShader);  // Attach the fragment shader
-    glLinkProgram(shaderProgram);  // Link the shaders into the program
-
-    int success;
-    char infoLog[512];
-    glGetShaderiv(shaderProgram, GL_LINK_STATUS, &success);  // Check for linking errors
-    if (!success) {
-        glGetShaderInfoLog(shaderProgram, 512, nullptr, infoLog);  // Get the error log
-        std::cerr << "Error while linking shaders\n" << infoLog << std::endl;
-    }
-
-    glDeleteShader(vertexShader);  // Delete the vertex shader as it's no longer needed
-    glDeleteShader(fragmentShader);  // Delete the fragment shader as it's no longer needed
-
-    return shaderProgram;
+GLuint createShaderProgram(const std::string &vertexShaderPath, const std::string &fragmentShaderPath)
+{
+	GLuint vertexShader = compileShader(vertexShaderPath, GL_VERTEX_SHADER);	   // Compile the vertex shader
+	GLuint fragmentShader = compileShader(fragmentShaderPath, GL_FRAGMENT_SHADER); // Compile the fragment shader
+
+	GLuint shaderProgram = glCreateProgram();	   // Create a shader program
+	glAttachShader(shaderProgram, vertexShader);   // Attach the vertex shader
+	glAttachShader(shaderProgram, fragmentShader); // Attach the fragment shader
+	glLinkProgram(shaderProgram);				   // Link the shaders into the program
+
+	int success;
+	char infoLog[512];
+	glGetShaderiv(shaderProgram, GL_LINK_STATUS, &success); // Check for linking errors
+	if (!success)
+	{
+		glGetShaderInfoLog(shaderProgram, 512, nullptr, infoLog); // Get the error log
+		std::cerr << "Error while linking shaders\n"
+				  << infoLog << std::endl;
+	}
+
+	glDeleteShader(vertexShader);	// Delete the vertex shader as it's no longer needed
+	glDeleteShader(fragmentShader); // Delete the fragment shader as it's no longer needed
+
+	return shaderProgram;
 }
 
 // Function to render the spheres
-void render(GLuint shaderProgram, GLuint vao, const std::vector<glm::vec3>& spherePositions) {
-    glUseProgram(shaderProgram);  // Use the shader program
+void render(GLuint shaderProgram, GLuint vao, const std::vector<Particle> &particles)
+{
+	glUseProgram(shaderProgram); // Use the shader program
 
-    // Set up the view matrix based on the camera's position, direction, and up vector
-    glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
-    // Set up the projection matrix with perspective projection
-    glm::mat4 projection = glm::perspective(glm::radians(fov), 1920.0f / 1080.0f, 0.1f, 100.0f);
+	// Set up the view matrix based on the camera's position, direction, and up vector
+	glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
+	// Set up the projection matrix with perspective projection
+	glm::mat4 projection = glm::perspective(glm::radians(fov), 1920.0f / 1080.0f, 0.1f, 100.0f);
 
-    // Pass the view matrix to the shader
-    GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
-    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
+	// Pass the view matrix to the shader
+	GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
+	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
 
-    // Pass the projection matrix to the shader
-    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
-    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
+	// Pass the projection matrix to the shader
+	GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
+	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
 
-    glBindVertexArray(vao);  // Bind the vertex array object
+	glBindVertexArray(vao); // Bind the vertex array object
 
-    // Loop through each sphere position and render the sphere
-    for (const auto& position : spherePositions) {
-        glm::mat4 model = glm::mat4(1.0f);  // Create the model matrix
-        GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
-        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));  // Pass the model matrix to the shader
+	// Loop through each sphere position and render the sphere
+	for (const auto &p : particles)
+	{
+		glm::mat4 model = glm::mat4(1.0f); // Create the model matrix
+		GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
+		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model)); // Pass the model matrix to the shader
 
-        GLint spherePosLoc = glGetUniformLocation(shaderProgram, "spherePosition");
-        glUniform3fv(spherePosLoc, 1, glm::value_ptr(position));  // Pass the sphere position to the shader
+		GLint spherePosLoc = glGetUniformLocation(shaderProgram, "spherePosition");
+		glUniform3fv(spherePosLoc, 1, glm::value_ptr(p.position)); // Pass the sphere position to the shader
 
-        glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0);  // Draw the sphere using the index buffer
-    }
+		glDrawElements(GL_TRIANGLES, sphereIndices.size(), GL_UNSIGNED_INT, 0); // Draw the sphere using the index buffer
+	}
 }
 
 // Function to clean up resources
-void cleanup(GLFWwindow* window, GLuint& shaderProgram, GLuint& vao) {
-    glDeleteProgram(shaderProgram);  // Delete the shader program
-    glDeleteVertexArrays(1, &vao);  // Delete the vertex array object
+void cleanup(GLFWwindow *window, GLuint &shaderProgram, GLuint &vao)
+{
+	glDeleteProgram(shaderProgram); // Delete the shader program
+	glDeleteVertexArrays(1, &vao);	// Delete the vertex array object
 
-    glfwDestroyWindow(window);  // Destroy the GLFW window
-    glfwTerminate();  // Terminate GLFW
+	glfwDestroyWindow(window); // Destroy the GLFW window
+	glfwTerminate();		   // Terminate GLFW
 }
 
 // Main function
-int main() {
-    // Initialize GLFW and create a window
-    GLFWwindow* window = initialize(1920, 1080, "OpenGL Sphere Drawing");
-
-    // Create the shader program
-    GLuint shaderProgram = createShaderProgram(
-        ASSETS_PATH"/shaders/test.vert.glsl",
-        ASSETS_PATH"/shaders/test.frag.glsl");
-
-    // Define parameters for the sphere
-    float radius = 1.0f;
-    int sectorCount = 36;
-    int stackCount = 18;
-
-    // Create buffers for the sphere
-    GLuint vao = createBuffersForSphere(radius, sectorCount, stackCount);
-
-    // Define positions for multiple spheres
-    std::vector<glm::vec3> spherePositions = {
-        {0.0f, 0.0f, 0.0f},
-        {2.0f, 2.0f, 0.0f},
-        {-2.0f, -2.0f, 0.0f},
-        {2.0f, -2.0f, 0.0f},
-        {-2.0f, 2.0f, 0.0f}
-    };
-
-    // Main render loop
-    while (!glfwWindowShouldClose(window)) {
-        // Calculate delta time
-        float currentFrame = glfwGetTime();
-        deltaTime = currentFrame - lastFrame;
-        lastFrame = currentFrame;
-
-        // Clear the color and depth buffers
-        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
-
-        // Render the spheres
-        render(shaderProgram, vao, spherePositions);
-
-        // Swap the front and back buffers
-        glfwSwapBuffers(window);
-        glfwPollEvents();  // Poll for and process events
-    }
-
-    // Clean up resources
-    cleanup(window, shaderProgram, vao);
-
-    // Program exits successfully
-    exit(EXIT_SUCCESS);
+int main()
+{
+	// Initialize GLFW and create a window
+	GLFWwindow *window = initialize(1920, 1080, "OpenGL Sphere Drawing");
+
+	// Create the shader program
+	GLuint shaderProgram = createShaderProgram(
+		ASSETS_PATH "/shaders/test.vert.glsl",
+		ASSETS_PATH "/shaders/test.frag.glsl");
+
+	// Create buffers for the sphere
+	GLuint vao = createBuffersForSphere(sphereRadius, sectorCount, stackCount);
+
+	// initialize globalParticleSystem
+	globalParticleSystem.initialize();
+
+	// Main render loop
+	while (!glfwWindowShouldClose(window))
+	{
+		// Calculate delta time
+		float currentFrame = glfwGetTime();
+		deltaTime = currentFrame - lastFrame;
+		lastFrame = currentFrame;
+
+		// update Particle
+		globalParticleSystem.step(deltaTime);
+
+		// Clear the color and depth buffers
+		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
+
+		// Render the spheres
+		render(shaderProgram, vao, globalParticleSystem.particles);
+
+		// Swap the front and back buffers
+		glfwSwapBuffers(window);
+		glfwPollEvents(); // Poll for and process events
+	}
+
+	// Clean up resources
+	cleanup(window, shaderProgram, vao);
+
+	// Program exits successfully
+	exit(EXIT_SUCCESS);
 }
diff --git a/src/pbd.h b/src/pbd.h
new file mode 100644
index 0000000..2bc5d8d
--- /dev/null
+++ b/src/pbd.h
@@ -0,0 +1,189 @@
+// misc
+#include <iostream>
+#include <vector>
+#include <cfloat>
+#include <thread>
+#include <algorithm>
+#include <unordered_map>
+
+#include <glm/glm.hpp>
+#include <glm/gtc/matrix_transform.hpp>
+#include <glm/gtc/type_ptr.hpp>
+
+// particle system related
+const float deltaT = 0.002f; // get the firrst frame started
+const bool globalEnableParticles = false;
+const glm::vec3 globalGravity = glm::vec3(0.0f, -0.05f, 0.0f);
+const int globalNumParticles = 1000;
+const float G = 2e-3f; // Gravitational constant, particle mass = 1.0f
+const float EpsilonGravity = 2e-2f;
+const float sphereRadius = 0.03f;              // for the spherical particles
+
+// fast random number generator based pcg32_fast
+#include <stdint.h>
+namespace PCG32
+{
+    static uint64_t mcg_state = 0xcafef00dd15ea5e5u; // must be odd
+    static uint64_t const multiplier = 6364136223846793005u;
+    uint32_t pcg32_fast(void)
+    {
+        uint64_t x = mcg_state;
+        const unsigned count = (unsigned)(x >> 61);
+        mcg_state = x * multiplier;
+        x ^= x >> 22;
+        return (uint32_t)(x >> (22 + count));
+    }
+    float rand()
+    {
+        return float(double(pcg32_fast()) / 4294967296.0);
+    }
+}
+
+// ====== implement it in A3 ======
+// fill in the missing parts
+
+class Particle
+{
+public:
+    glm::vec3 position = glm::vec3(0.0f);
+    glm::vec3 velocity = glm::vec3(0.0f);
+    glm::vec3 prevPosition = position;
+
+    glm::vec3 force = glm::vec3(0.0f); // Force accumulator
+    float mass = 1.0f;                 // Particle mass
+    float radius = sphereRadius;              // for the spherical particles
+
+    bool resolveCollision(Particle &other)
+    {
+        glm::vec3 diff = position - other.position;
+        float dist = glm::distance(position, other.position);
+        float minDist = radius + other.radius;
+
+        //printf("dist = %f, minDist = %f\n", dist, minDist);
+        if (dist < minDist)
+        {
+            glm::vec3 collisionNormal = glm::normalize(diff);
+            float penetrationDepth = minDist - dist;
+
+            // Move particles apart to resolve the collision
+            // half for each
+            position += 0.5f * penetrationDepth * collisionNormal;
+            other.position -= 0.5f * penetrationDepth * collisionNormal;
+
+            // Calculate relative velocity`
+            glm::vec3 relativeVelocity = velocity - other.velocity;
+
+            // Calculate velocity along the collision normal
+            float collisionImpulse = dot(relativeVelocity, collisionNormal);
+
+            // Apply collision impulse
+            if (collisionImpulse < 0)
+            {
+                force -= (collisionImpulse / mass) * collisionNormal;
+                other.force += (collisionImpulse / other.mass) * collisionNormal;
+            }
+            //printf("collision!\n");
+            return true;
+        }
+        //printf("no collision!\n");
+        return false;
+    }
+
+    void reset()
+    {
+        position = glm::vec3(PCG32::rand(), PCG32::rand(), PCG32::rand()) - float(0.5f);
+        velocity = 2.0f * glm::vec3((PCG32::rand() - 0.5f), 0.0f, (PCG32::rand() - 0.5f));
+
+        prevPosition = position;
+        position += velocity * deltaT;
+    }
+
+    void step(float deltaTime)
+    {
+        // Save the current position
+        glm::vec3 temp = position;
+
+        // Collision box [-0.5, 0.5] × [-0.5, 0.5] × [-0.5, 0.5]
+		for (int axis = 0; axis < 3; ++axis)
+		{
+			float clampAmount = position[axis];
+			// save the clamp amount for position,
+			// gonna apply the same amount to prevPosition as well
+
+			if (position[axis] > 0.5f)
+			{
+				// Reflect prevPosition
+				temp[axis] = 2.0f * position[axis] - temp[axis];
+				// Move position to the boundary]
+				position[axis] = 0.5f;
+			}
+			else if (position[axis] < -0.5f)
+			{
+				// Reflect prevPosition
+				temp[axis] = 2.0f * position[axis] - temp[axis];
+				// Move position to the boundary
+				position[axis] = -0.5f;
+			}
+
+			// move the prevPosition by the same amount
+			clampAmount -= position[axis];
+			temp[axis] -= clampAmount;
+		}
+
+        // Calculate the new position using Verlet integration
+        glm::vec3 acceleration = force / mass;
+        position = position + (position - prevPosition) + (acceleration + globalGravity) * (deltaTime * deltaTime);
+
+        // Velocity
+        // v(t + dt) = (r(t + dt) - r(t - dt)) / (2 * dt)
+        velocity = (position - temp) / (2.0f * deltaTime);
+
+        // Update the previous position
+        prevPosition = temp;
+    }
+};
+
+class ParticleSystem
+{
+public:
+    std::vector<Particle> particles;
+    float sphereSize = 0.0f;
+
+    ParticleSystem() {};
+
+    void initialize()
+    {
+        particles.resize(globalNumParticles);
+
+        for (int i = 0; i < globalNumParticles; i++)
+        {
+            particles[i].reset();
+        }
+    }
+
+    void step(float deltaTime)
+    {
+
+        for (auto &particle : particles)
+        {
+            particle.force = glm::vec3(0.0f); // Reset force accumulator
+        }
+        // Handle collisions
+        for (int i = 0; i < globalNumParticles; i++)
+        {
+            for (int j = i + 1; j < globalNumParticles; j++)
+            {
+                particles[i].resolveCollision(particles[j]);
+            }
+        }
+
+        // add some particle-particle interaction here
+        // spherical particles can be implemented here
+        for (int i = 0; i < globalNumParticles; i++)
+        {
+            particles[i].step(deltaTime);
+        }
+    }
+};
+
+static ParticleSystem globalParticleSystem;
