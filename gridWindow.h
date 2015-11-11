#ifndef GRID_WINDOW_H
#define GRID_WINDOW_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>

void initializeGridWindow();
void updateGridWindow(int gridSize, double **grid);
void shutdownGridWindow();

bool hasExitedGridWindow();

extern GLFWwindow *gridWindow;

#endif