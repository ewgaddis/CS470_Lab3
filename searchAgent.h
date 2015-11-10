#ifndef SEARCH_AGENT_H
#define SEARCH_AGENT_H

#include "team.h"
#include "potentialFields.h"
#include "graphAlgorithms.h"

class SearchAgent {
	BZRC* myTeam;
	deque<Vector> path;
	int botIndex;
	Vector* curVector;
	Vector oldVector;
	base_t* colorBase;
	string color;
	Vector* baseCenter;
	double oldAngle;
	double maxDist;
public: SearchAgent(BZRC* team, int index, const Graph &g, GraphSearch *s);

		void Update(string color);
private:
	boolean isInBase(base_t* base, flag_t* flag);
	void setBaseCenter(base_t* base);
	boolean isCloseToGoal(Vector location, Vector goal);
};
#endif