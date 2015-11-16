#include "occGrid.h"

#include "geometry.h"

#include <iostream>

using namespace std;

double OCCGrid::getLikelihood(bool o, bool s)
{
	if(s)
	{
		return (o ? truePositive : 1.0 - truePositive);
	}

	return (o ? 1.0 - trueNegative : trueNegative);
}

double OCCGrid::getNormalizer(bool o, int i, int j)
{
	return getLikelihood(o, true) * grid[j][i] + getLikelihood(o, false) * (1.0 - grid[j][i]);
}

bool OCCGrid::isPointInObstacle(int i, int j,
								const vector<obstacle_t> *obstacles,
								int *newI) const
{
	Vector p(i - halfGridSize, j - halfGridSize);

	vector<obstacle_t>::const_iterator itObstacle = obstacles->begin();
	while(itObstacle != obstacles->end())
	{
		const obstacle_t obstacle = (*itObstacle);

		if(isPointWithinObstacle(p, Vector(obstacle.o_corner[0][0], obstacle.o_corner[0][1]),
									Vector(obstacle.o_corner[1][0], obstacle.o_corner[1][1]),
									Vector(obstacle.o_corner[3][0], obstacle.o_corner[3][1])))
		{
			*newI = (int)obstacle.o_corner[1][0] + halfGridSize;
			return true;
		}

		++itObstacle;
	}

	return false;
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
		int i = halfGridSize + x + r;

		if(i < 0 || i >= gridSize)
			continue;

		const string & row = sensorGrid[r];

		int rowLength = (int)row.length();

		for(int c = rowLength - 1; c >= 0; --c)
		{
			int j = halfGridSize + y + c;

			if(j < 0 || j >= gridSize)
				continue;

			bool o = (row[c] == '1');

			grid[j][i] = getLikelihood(o, true) * grid[j][i] / getNormalizer(o, i, j);
		}
	}
}

void OCCGrid::getObstacles(vector<obstacle_t> *obstacles,
						   double occThreshold,
						   int minWidth, int minHeight)
{
	if(!obstacles)
	{
		return;
	}

	for(int j = 0; j < gridSize; ++j)
	{
		for(int i = 0; i < gridSize; ++i)
		{
			if(grid[j][i] >= occThreshold)
			{
				if(isPointInObstacle(i, j, obstacles, &i))
				{
					continue;
				}

				int i1 = i + 1;
				int temp = i1;

				while(grid[j][i1] >= occThreshold &&
					i1 < gridSize &&
					!isPointInObstacle(i1, j, obstacles, &temp))
				{
					++i1;
				}

				if(i1 < i + minWidth)
				{
					i = temp;
					continue;
				}

				int j1 = j + 1;

				while(grid[j1][i] >= occThreshold && j1 < gridSize)
				{
					++j1;
				}

				if(j1 < j + minHeight)
				{
					continue;
				}

				int j2 = j + 1;

				while(grid[j2][i1 - 1] >= occThreshold && j2 < gridSize)
				{
					++j2;
				}

				int j3 = j1;

				if(j2 >= j + minHeight && j2 < j1)
				{
					j3 = j2;
				}

				double x = i - halfGridSize;
				double y = j - halfGridSize;

				obstacle_t obstacle;
				obstacle.numCorners = 4;

				obstacle.o_corner[0][0] = x;
				obstacle.o_corner[0][1] = y;

				obstacle.o_corner[1][0] = i1 - halfGridSize;
				obstacle.o_corner[1][1] = y;

				obstacle.o_corner[2][0] = i1 - halfGridSize;
				obstacle.o_corner[2][1] = j3 - halfGridSize;

				obstacle.o_corner[3][0] = x;
				obstacle.o_corner[3][1] = j3 - halfGridSize;

				obstacles->push_back(obstacle);

				i = i1;
			}
		}
	}
}