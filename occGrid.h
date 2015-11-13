#ifndef OCC_GRID_H
#define OCC_GRID_H

class BZRC;

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

public:
	OCCGrid(BZRC *t, int size, double prior,
			double truePos, double trueNeg);
	~OCCGrid();

	void update(int tank);

	int    getGridSize() const { return gridSize; }
	double **getGrid()         { return grid;     }
};

#endif