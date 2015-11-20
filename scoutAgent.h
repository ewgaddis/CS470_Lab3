#ifndef SCOUT_AGENT_H
#define SCOUT_AGENT_H

#include "team.h"
#include "potentialFields.h"
#include "graphAlgorithms.h"
#include "occGrid.h"

class ScoutAgent {
	BZRC* myTeam;
	deque<Vector> path;
	int botIndex;
	Vector* curVector;
	Vector oldVector;
	double oldAngle;
	double maxDist;
	int time;
	int maxtime;
	Vector oldLoc;
	string myArea;
	//Graph* graph;
	//ASearch *search;
	//int numObstacles;
public: ScoutAgent(BZRC* team, int index, string area);

		void Update(vector <obstacle_t> obstacles, OCCGrid * grid);
private:
	boolean isCloseToGoal(Vector location, Vector goal);
	//void recalculatePath(Vector curGoal, vector<tank_t>myTanks, vector<obstacle_t> obstacles);
	boolean isInObstacle(Vector pos, vector<obstacle_t> obstacles);
	Vector * findNextGoal(OCCGrid * grid);
	Vector * setNextGoal(int x, int y);
};
#endif