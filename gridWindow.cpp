#include "gridWindow.h"

#include <iostream>

using namespace std;

#define WINDOW_SIZE 400

GLFWwindow *gridWindow = 0;

void initializeGridWindow()
{
	cout << "Initializing grid window..." << endl;

	if(!glfwInit())
	{
		cout << "Failed to initialize GLFW" << endl;
		return;
	}

	gridWindow = glfwCreateWindow(WINDOW_SIZE, WINDOW_SIZE,
								  "Grid", NULL, NULL);

	if(!gridWindow)
	{
		cout << "Failed to create grid window" << endl;

		glfwTerminate();
		return;
	}

	glfwMakeContextCurrent(gridWindow);

	GLenum err = glewInit();

	if(err != GLEW_OK)
	{
		cout << "Failed to initialize GLEW" << endl;

		glfwDestroyWindow(gridWindow);
		gridWindow = 0;

		glfwTerminate();

		return;
	}

	glClearColor(0.0, 0.0, 0.0, 0.0);
}

void updateGridWindow(int gridSize, double **grid)
{
	if(gridWindow == 0)
	{
		return;
	}

	glClear(GL_COLOR_BUFFER_BIT);

	GLubyte frameBuffer[WINDOW_SIZE * WINDOW_SIZE * 3];

	int denom = WINDOW_SIZE / gridSize;

	for(int j = 0; j < WINDOW_SIZE; ++j)
	{
		for(int i = 0; i < WINDOW_SIZE; ++i)
		{
			GLubyte value = (GLubyte)(grid[j / denom][i / denom] * 255);

			frameBuffer[(j * WINDOW_SIZE + i) * 3    ] = value;
			frameBuffer[(j * WINDOW_SIZE + i) * 3 + 1] = value;
			frameBuffer[(j * WINDOW_SIZE + i) * 3 + 2] = value;
		}
	}

	glDrawPixels(WINDOW_SIZE, WINDOW_SIZE, GL_RGB,
				 GL_UNSIGNED_BYTE, frameBuffer);

	glfwSwapBuffers(gridWindow);
		
	glfwPollEvents();
}

void shutdownGridWindow()
{
	if(gridWindow)
	{
		cout << "Shutting down grid window..." << endl;
		glfwDestroyWindow(gridWindow);
		gridWindow = 0;

		glfwTerminate();
	}
}

bool hasExitedGridWindow()
{
	if(!gridWindow)
	{
		return false;
	}

	return glfwWindowShouldClose(gridWindow) != 0;
}