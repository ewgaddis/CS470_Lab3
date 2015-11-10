#include "dumbAgent.h"

DumbAgent::DumbAgent(BZRC* team,int index){
	myTeam = team;
	botIndex = index;
}

void DumbAgent::Update(){
	int driveTime = rand() % 8000 + 3000;
	//int driveTime = 2000;

	myTeam->speed(botIndex, 1);
	while (driveTime > 0){
		Sleep(500);
		if (driveTime%2000 == 0)
			myTeam->shoot(botIndex);
		driveTime -= 500;
	}
	//Sleep(2000);
	//myTeam->shoot(botIndex);
	//Sleep(1000);
	myTeam->speed(botIndex, 0);
	Sleep(500);
	myTeam->angvel(botIndex, 1);
	Sleep(1650); //turns 60 degrees now
	myTeam->angvel(botIndex, 0);
	myTeam->shoot(botIndex);
	//Sleep(500);
}