#include "gridWindow.h"

#include <iostream>

using namespace std;

#define WINDOW_SIZE 800

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

void updateGridWindow(int gridSize, double **grid,
					  const vector<obstacle_t> *obstacles)
{
	if(gridWindow == 0)
	{
		return;
	}

	glClear(GL_COLOR_BUFFER_BIT);

	GLubyte *frameBuffer = new GLubyte[WINDOW_SIZE * WINDOW_SIZE * 3];

	int denom = WINDOW_SIZE / gridSize;

	for(int j = 0; j < WINDOW_SIZE; ++j)
	{
		int r = j / denom;

		for(int i = 0; i < WINDOW_SIZE; ++i)
		{
			GLubyte value = (GLubyte)(grid[r][i] * 255);

			frameBuffer[(j * WINDOW_SIZE + i) * 3    ] = value;
			frameBuffer[(j * WINDOW_SIZE + i) * 3 + 1] = value;
			frameBuffer[(j * WINDOW_SIZE + i) * 3 + 2] = value;
		}
	}

	if(obstacles)
	{
		vector<obstacle_t>::const_iterator itObstacle = obstacles->begin();
		while(itObstacle != obstacles->end())
		{
			const obstacle_t &obstacle = (*itObstacle);

			for(int c = 0; c < obstacle.numCorners; ++c)
			{
				int i = (int)obstacle.o_corner[c][0] + 400;
				int j = (int)obstacle.o_corner[c][1] + 400;

				for(int j1 = max(j - 3, 0); j1 < j + 3 && j1 < WINDOW_SIZE; ++j1)
				{
					for(int i1 = max(i - 3, 0); i1 < i + 3 && i1 < WINDOW_SIZE; ++i1)
					{
						frameBuffer[(j1 * WINDOW_SIZE + i1) * 3    ] = 127 + (int)(128.0f * ((float)c / (float)(obstacle.numCorners - 1)));
						frameBuffer[(j1 * WINDOW_SIZE + i1) * 3 + 1] = 0;
						frameBuffer[(j1 * WINDOW_SIZE + i1) * 3 + 2] = 0;
					}
				}
			}

			++itObstacle;
		}
	}

	glDrawPixels(WINDOW_SIZE, WINDOW_SIZE, GL_RGB,
				 GL_UNSIGNED_BYTE, frameBuffer);

	glfwSwapBuffers(gridWindow);
		
	glfwPollEvents();

	delete[] frameBuffer;
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