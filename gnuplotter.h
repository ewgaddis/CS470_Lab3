#ifndef GNUPLOTTER_H
#define GNUPLOTTER_H

#include "team.h"
#include "geometry.h"
#include "graph.h"
#include "graphAlgorithms.h"

#include <vector>

class GNUPlotter
{
private:
	FILE *file;

public:
	GNUPlotter();

	void createFile(const char *fileName, const char *title);
	void finishFile();

	void drawArrow(double x, double y, const Vector &v, int color);
	void drawArrow(const Vector &t, const Vector &h, int color);
	void drawLine(double x1, double y1, double x2, double y2, int color);
	void drawCircle(const Vector &c, int radius,
					unsigned char r, unsigned char g, unsigned char b);
	void drawText(double x, double y, const char *text);

	void drawObstacles(const std::vector<obstacle_t> &obstacles);

	void drawGraph(const Graph &graph);

	void drawGraphSearch(const Graph &graph,
						 const GraphSearch *graphSearch);
};

#endif