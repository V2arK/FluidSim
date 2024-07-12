#include <fstream>
#include <iostream>
#include <cstdlib>
#include <string>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Camera parameters
glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
float yaw = -90.0f;
float pitch = 0.0f;
float lastX = 960.0f, lastY = 540.0f;
bool firstMouse = true;
float fov = 45.0f;
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// Movement speeds
float movementSpeed = 10.0f;
float mouseSensitivity = 0.1f;

//________________________________________________CALLBACK_FUNCTIONS_________________________________________________//

static void errorCallback(int error, const char* description) {
	std::cerr << description;
}

static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    // close window when ESC has been pressed
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
    float cameraSpeed = movementSpeed * deltaTime;
    if (key == GLFW_KEY_W)
        cameraPos += cameraSpeed * cameraFront;
    if (key == GLFW_KEY_S)
        cameraPos -= cameraSpeed * cameraFront;
    if (key == GLFW_KEY_A)
        cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
    if (key == GLFW_KEY_D)
        cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
}

static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {}

static void mousePositionCallback(GLFWwindow* window, double xpos, double ypos) {
    if (firstMouse) {
        lastX = xpos;
        lastY = ypos;
        firstMouse = false;
    }

    float xoffset = xpos - lastX;
    float yoffset = lastY - ypos; // reversed since y-coordinates range from bottom to top
    lastX = xpos;
    lastY = ypos;

    xoffset *= mouseSensitivity;
    yoffset *= mouseSensitivity;

    yaw += xoffset;
    pitch += yoffset;

    if (pitch > 89.0f)
        pitch = 89.0f;
    if (pitch < -89.0f)
        pitch = -89.0f;

    glm::vec3 front;
    front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
    front.y = sin(glm::radians(pitch));
    front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
    cameraFront = glm::normalize(front);
}

static void framebufferSizeCallback(GLFWwindow* window, int w, int h) { glViewport(0, 0, w, h); }

//_________________________________________________INITIALIZATION____________________________________________________//

GLFWwindow* initialize(int width, int height, const std::string& title) {
	GLFWwindow* window;
	glfwSetErrorCallback(errorCallback);

	// initialize glfw window
	if (!glfwInit())
		exit(EXIT_FAILURE);

	// we want to use the opengl 4.6 core profile
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// actually create the window
	window = glfwCreateWindow(width, height, title.c_str(), nullptr, nullptr);

	// make sure the window creation was successful
	if (!window) {
		std::cerr << "Failed to open GLFW window.\n";
		glfwTerminate();
		exit(EXIT_FAILURE);
	}
	glfwMakeContextCurrent(window);

	// initialize glad for loading all OpenGL functions
	if (!gladLoadGL((GLADloadfunc)glfwGetProcAddress)) {
		std::cerr << "Something went wrong!\n";
		exit(EXIT_FAILURE);
	}

	// print some information about the supported OpenGL version
	const GLubyte* renderer = glGetString(GL_RENDERER);
	const GLubyte* version = glGetString(GL_VERSION);
	std::cout << "Renderer: " << renderer << '\n';
	std::cout << "OpenGL version supported " << version << '\n';

	// register user callbacks
	glfwSetKeyCallback(window, keyCallback);
	glfwSetMouseButtonCallback(window, mouseButtonCallback);
	glfwSetCursorPosCallback(window, mousePositionCallback);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

	// set the clear color of the window
	glClearColor(0.3f, 0.3f, 0.3f, 0.0f);

	// Hide the mouse cursor and capture it within the window
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	return window;
}

GLuint createBuffers() {
	// specify the layout of the vertex data, being the vertex position followed by the vertex color
	struct Vertex {
		glm::vec3 pos;
		glm::vec3 color;
	};

	// we specify a triangle with red, green, blue at the tips of the triangle
	Vertex vertexData[] = {
		Vertex{glm::vec3(-0.5f, -0.5f, 0.0f), glm::vec3(1.f, 0.f, 0.f)},
		Vertex{glm::vec3( 0.5f, -0.5f, 0.0f), glm::vec3(0.f, 1.f, 0.f)},
		Vertex{glm::vec3( 0.0f,  0.5f, 0.0f), glm::vec3(0.f, 0.f, 1.f)}
	};

	// create the vertex array object that holds all vertex buffers
	GLuint vao;
	glGenVertexArrays(1, &vao);
	glBindVertexArray(vao);

	// create a vertex buffer that contains all vertex positions and copy the vertex positions into that buffer
	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertexData), vertexData, GL_STATIC_DRAW);

	// we need to tell the buffer in which format the data is and we need to explicitly enable it
	// first we specify the layout of the vertex position
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(struct Vertex, pos)));
	glEnableVertexAttribArray(0);
	// then we specify the layout of the vertex color
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)(offsetof(struct Vertex, color)));
	glEnableVertexAttribArray(1);

	return vao;
}

GLuint compileShader(const std::string& path, GLenum shaderType) {
	// grab the contents of the file and store the source code in a string
	std::ifstream filestream(path);
	std::string shaderSource((std::istreambuf_iterator<char>(filestream)),
		std::istreambuf_iterator<char>());

	// create and compile the shader
	GLuint shaderHandle = glCreateShader(shaderType);
	const char* shaderSourcePtr = shaderSource.c_str();
	glShaderSource(shaderHandle, 1, &shaderSourcePtr, nullptr);
	glCompileShader(shaderHandle);

	// check if compilation was successful
	int  success;
	char infoLog[512];
	glGetShaderiv(shaderHandle, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(shaderHandle, 512, nullptr, infoLog);
		std::cerr << "Error while compiling shader\n" << infoLog << std::endl;
	}

	// return the shader handle
	return shaderHandle;
}

GLuint createShaderProgram(const std::string& vertexShaderPath, const std::string& fragmentShaderPath) {
	// create and compile shaders
	GLenum vertexShader = compileShader(vertexShaderPath, GL_VERTEX_SHADER);
	GLenum fragmentShader = compileShader(fragmentShaderPath, GL_FRAGMENT_SHADER);

	// create a shader program, attach both shaders and link them together
	GLuint shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);

	// check for errors while linking the shaders together
	int  success;
	char infoLog[512];
	glGetShaderiv(shaderProgram, GL_COMPILE_STATUS, &success);
	if (!success) {
		glGetShaderInfoLog(shaderProgram, 512, nullptr, infoLog);
		std::cerr << "Error while linking shaders\n" << infoLog << std::endl;
	}

	// after creating the shader program we don't need the two shaders anymore
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);

	// return the shader program handle
	return shaderProgram;
}

//______________________________________________________RENDER_______________________________________________________//

void render(GLuint shaderProgram, GLuint vao) {
	glUseProgram(shaderProgram);

    // Set up view and projection matrices
    glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
    glm::mat4 projection = glm::perspective(glm::radians(fov), 1920.0f / 1080.0f, 0.1f, 100.0f);

    // Send the matrices to the shader
    GLint viewLoc = glGetUniformLocation(shaderProgram, "view");
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

    GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));

    // Model matrix
    glm::mat4 model = glm::mat4(1.0f);
    GLint modelLoc = glGetUniformLocation(shaderProgram, "model");
    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));

    glBindVertexArray(vao);
    glDrawArrays(GL_TRIANGLES, 0, 3);
}

//______________________________________________________CLEANUP______________________________________________________//

void cleanup(GLFWwindow* window, GLuint& shaderProgram, GLuint& vao) {
	// do some custom cleanup here
	glDeleteProgram(shaderProgram);
	glDeleteVertexArrays(1, &vao);

	// lastly destroy the window and terminate glfw
	glfwDestroyWindow(window);
	glfwTerminate();
}

//_______________________________________________________MAIN________________________________________________________//

int main() {
	// create a window with the specified width, height and title and initialize OpenGL
	GLFWwindow* window = initialize(1920, 1080, "OpenGL Starter Project");
	GLuint shaderProgram = createShaderProgram(
		ASSETS_PATH"/shaders/test.vert.glsl",
		ASSETS_PATH"/shaders/test.frag.glsl");
	GLuint vao = createBuffers();

	// loop until the user presses ESC or the window is closed programmatically
	while (!glfwWindowShouldClose(window)) {
		// calculate delta time
        float currentFrame = glfwGetTime();
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

		// clear the back buffer with the specified color and the depth buffer with 1
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// render to back buffer
		render(shaderProgram, vao);

		// switch front and back buffers
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	// clean up all created objects
	cleanup(window, shaderProgram, vao);

	// program exits properly
	exit(EXIT_SUCCESS);
}