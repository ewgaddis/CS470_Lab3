#ifndef OCC_GRID_H
#define OCC_GRID_H

#include "team.h"

#include <vector>

class OCCGrid
{
private:
	BZRC *team;

	int gridSize;
	int halfGridSize;
	double **grid;

	double truePositive;
	double trueNegative;

private:
	double getLikelihood(bool o, bool s);
	double getNormalizer(bool o, int i, int j);

	bool isPointInObstacle(int i, int j,
						   const std::vector<obstacle_t> *obstacles,
						   int *newI) const;

public:
	OCCGrid(BZRC *t, int size, double prior,
			double truePos, double trueNeg);
	~OCCGrid();

	void update(int tank);

	void getObstacles(std::vector<obstacle_t> *obstacles,
					  double occThreshold,
					  int minWidth, int minHeight);

	int    getGridSize() const { return gridSize; }
	double **getGrid()         { return grid;     }
};

#endif