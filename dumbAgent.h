#include "team.h"

class DumbAgent {
	BZRC* myTeam;
	int botIndex;
public: DumbAgent(BZRC* team, int index);

		void Update();
};