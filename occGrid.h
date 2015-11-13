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

public:
	OCCGrid(BZRC *t, int size, double prior);
	~OCCGrid();

	void update(int tank);

	int    getGridSize() const { return gridSize; }
	double **getGrid()         { return grid;     }
};

#endif