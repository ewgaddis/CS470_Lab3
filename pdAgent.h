#ifndef PD_AGENT_H
#define PD_AGENT_H

#include "team.h"
#include "potentialFields.h"

class PDAgent {
	BZRC* myTeam;
	int botIndex;
	Vector* curVector;
	Vector oldVector;
	base_t* colorBase;
	string color;
	Vector* baseCenter;
	double oldAngle;
public: PDAgent(BZRC* team, int index);

		void Update(string color);
private:
	boolean isInBase(base_t* base, flag_t* flag);
	void setBaseCenter(base_t* base);
};
#endif