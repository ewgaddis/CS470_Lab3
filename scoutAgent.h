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
public: ScoutAgent(BZRC* team, int index, string area);

		void Update(vector <obstacle_t> obstacles, OCCGrid * grid);
private:
	boolean isCloseToGoal(Vector location, Vector goal);
};
#endif