#include "team.h"

#include "gnuplotter.h"
#include "potentialFields.h"
#include "graph.h"
#include "graphAlgorithms.h"
#include "dumbAgent.h"
#include "pdAgent.h"
#include "searchAgent.h"
#include "gridWindow.h"
#include "occGrid.h"

#include <vector>
#include <conio.h>

using namespace std;

BZRC *team;

OCCGrid *grid;

void world_init(BZRC *my_team)
{
	team = my_team;

	grid = new OCCGrid(team, 800, 0.5);
}

void robot_pre_update()
{
}

bool robot_update()
{
	grid->update(0);

	updateGridWindow(grid->getGridSize(),
					 grid->getGrid());

	bool exited = hasExitedGridWindow();

	if(exited)
	{
		delete grid;

		return false;
	}

	return true;
}

void robot_post_update()
{
}