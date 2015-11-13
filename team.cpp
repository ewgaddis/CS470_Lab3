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
#include "scoutAgent.h"

#include <vector>
#include <conio.h>

using namespace std;

BZRC *team;

OCCGrid *grid;
ScoutAgent *scout1;
ScoutAgent *scout2;
double trueNeg;
double truePos;

void world_init(BZRC *my_team)
{
	team = my_team;
	vector<constant_t> constants;
	team->get_constants(&constants);
	for each(constant_t c in constants){
		if (c.name.compare("truenegative") == 0){
			trueNeg = atof(c.value.c_str());
			//printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		else if (c.name.compare("truepositive") == 0){
			trueNeg = atof(c.value.c_str());
			//printf("name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		}
		//printf("1name: %s value: %s \n", c.name.c_str(), c.value.c_str());
		//printf(c.name.c_str());
	}
	scout1 = new ScoutAgent(team, 0, "upper");

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
		delete scout1;
		delete grid;

		return false;
	}

	scout1->Update();

	return true;
}

void robot_post_update()
{
}