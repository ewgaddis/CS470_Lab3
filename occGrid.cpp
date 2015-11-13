#include "occGrid.h"

#include "team.h"

#include <iostream>

using namespace std;

OCCGrid::OCCGrid(BZRC *t,
				 int size,
				 double prior) : team(t),
								 gridSize(size),
								 halfGridSize(size / 2)
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

	for(int j = 0; j < gridSize; ++j)
	{
		for(int i = 0; i < gridSize; ++i)
		{
			grid[j][i] = 0.5;
		}
	}

	for(int r = 0; r < (int)sensorGrid.size(); ++r)
	{
		const string & row = sensorGrid[r];

		int rowLength = (int)row.length();

		for(int c = rowLength - 1; c >= 0; --c)
		{
			int i = halfGridSize + x + r;
			int j = halfGridSize + y + c;

			if(row[c] == '0')
			{
				grid[j][i] = 0.0;
			}
			else
			{
				grid[j][i] = 1.0;
			}
		}
	}
}