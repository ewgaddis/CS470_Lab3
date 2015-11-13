#include "team.h"

#include "gnuplotter.h"
#include "potentialFields.h"
#include "graph.h"
#include "graphAlgorithms.h"
#include "dumbAgent.h"
#include "pdAgent.h"
#include "searchAgent.h"
#include "gridWindow.h"
#include "scoutAgent.h"

#include <vector>
#include <conio.h>

using namespace std;

BZRC *team;
DumbAgent *dumb1;
DumbAgent *dumb2;
DumbAgent *dumb3;
PDAgent *pd1;
vector <PDAgent*> agents;
SearchAgent *search1;
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
	//dumb1 = new DumbAgent(team, 0);
	//dumb2 = new DumbAgent(team, 1);
	//dumb3 = new DumbAgent(team, 2);
	//pd1 = new PDAgent(team, 0);
	/*agents = vector<PDAgent*>();
	for (int i = 0; i < 3; i++)
	{
		agents.push_back(new PDAgent(team, i));
		//agents.at(i) = new PDAgent(team, i);
	}*/

	/*vector<tank_t> tanks;
	team->get_mytanks(&tanks);

	vector<flag_t> flags;
	team->get_flags(&flags);

	vector<obstacle_t> obstacles;
	team->get_obstacles(&obstacles);

	Graph graph;

	createVisibilityGraph(Vector(tanks.at(0).pos[0], tanks.at(0).pos[1]),
		Vector(flags.at(0).pos[0], flags.at(0).pos[1]),
		obstacles, &graph);

	drawGraphSearch(graph, new ASearch(graph), 100,
					"./Data/AStar/special",
					"A* Search - Iteration #");




	search1 = new SearchAgent(team, 0,graph,new ASearch(graph));*/
	scout1 = new ScoutAgent(team,0,"upper");

}

void robot_pre_update()
{
}

bool robot_update()
{
	double **grid = new double*[50];

	for(int j = 0; j < 50; ++j)
	{
		grid[j] = new double[50];

		for(int i = 0; i < 50; ++i)
		{
			grid[j][i] = (rand() % 101) / 100.0;
		}
	}

	grid[0][0] = grid[0][1] = grid[0][2] = 1;
	grid[1][0] = grid[1][1] = grid[1][2] = 1;

	updateGridWindow(50, grid);

	for(int j = 0; j < 50; ++j)
	{
		delete[] grid[j];
	}

	delete[] grid;
	scout1->Update();
	//bzrflag --window-size=800x600 --default-true-positive=.97 --default-true-negative=.9 --occgrid-width=100 --no-report-obstacles
	//notes for self: get robot exploring grid, get it to go around obstacles, set waypoints to next gray spot. enjoy wandering. repulse from each spot?
	//truepositive/truenegative
	//dumb1->Update();
	//dumb2->Update();
	//dumb3->Update();
	//pd1->Update();
	//search1->Update("blue");
	/*for (int i = 0; i < agents.size(); i++)
	{
		string color;
		if (i % 3 == 0){
			color = "red";
		}
		else if (i % 3 == 1)
			color = "blue";
		if (i % 3 == 2)
			color = "purple";

		agents.at(i)->Update(color);
	}*/
	/*vector<tank_t> tanks;
	team->get_mytanks(&tanks);

	vector<flag_t> flags;
	team->get_flags(&flags);

	vector<obstacle_t> obstacles;
	team->get_obstacles(&obstacles);

	Graph graph;
	
	createVisibilityGraph(Vector(tanks.at(0).pos[0], tanks.at(0).pos[1]),
						  Vector(flags.at(0).pos[0], flags.at(0).pos[1]),
						  obstacles, &graph);

	drawGraphSearch(graph, new DFSearch(graph), 100,
					"./Data/DFS2/dfs",
					"Depth-First Search - Iteration #");*/
	return !hasExitedGridWindow();
}

void robot_post_update()
{
}