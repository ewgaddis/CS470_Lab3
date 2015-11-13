#include "occGrid.h"

#include "team.h"

#include <iostream>

using namespace std;

double OCCGrid::getLikelihood(bool o, bool s)
{
	if(s)
	{
		return (o ? truePositive : 1.0 - trueNegative);
	}

	return (o ? 1.0 - truePositive : trueNegative);
}

double OCCGrid::getNormalizer(bool o, int i, int j)
{
	return getLikelihood(o, true) * grid[j][i] + getLikelihood(o, false) * (1.0 - grid[j][i]);
}

OCCGrid::OCCGrid(BZRC *t,
				 int size,
				 double prior,
				 double truePos,
				 double trueNeg) : team(t),
								   gridSize(size),
								   halfGridSize(size / 2),
								   truePositive(truePos),
								   trueNegative(trueNeg)
{
	cout << "Creating occupancy grid of size " << gridSize << "x" << gridSize << "..." << endl;
	cout << "Initial prior value: " << prior << endl;

	grid = new double*[gridSize];

	for(int j = 0; j < gridSize; ++j)
	{
		grid[j] = new double[gridSize];
		
		for(int i = 0; i < gridSize; ++i)
		{
			grid[j][i] = prior;
		}
	}
}

OCCGrid::~OCCGrid()
{
	cout << "Deleting occupancy grid..." << endl;

	for(int j = 0; j < gridSize; ++j)
	{
		delete[] grid[j];
	}

	delete[] grid;
}

void OCCGrid::update(int tank)
{
	vector<string> sensorGrid;
	int x, y;

	team->getOCCGrid(tank, &x, &y, &sensorGrid);

	for(int r = 0; r < (int)sensorGrid.size(); ++r)
	{
		const string & row = sensorGrid[r];

		int rowLength = (int)row.length();

		for(int c = rowLength - 1; c >= 0; --c)
		{
			int i = halfGridSize + x + r;
			int j = halfGridSize + y + c;

			bool o = (row[c] == '1');

			grid[j][i] = getLikelihood(o, true) * grid[j][i] / getNormalizer(o, i, j);
		}
	}
}