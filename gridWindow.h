#ifndef GRID_WINDOW_H
#define GRID_WINDOW_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "team.h"

#include <vector>

void initializeGridWindow();
void updateGridWindow(int gridSize, double **grid,
					  const std::vector<obstacle_t> *obstacles);
void shutdownGridWindow();

bool hasExitedGridWindow();

extern GLFWwindow *gridWindow;

#endif