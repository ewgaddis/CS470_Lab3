#include "pdAgent.h"
#include "geometry.h"
#include <math.h>

PDAgent::PDAgent(BZRC* team, int index){
	myTeam = team;
	botIndex = index;
	curVector = new Vector();
	oldVector = Vector();
	vector<base_t> bases;
	base_t* colorBase;
	vector <tank_t> myTanks;
	oldAngle = 0.0;
	myTeam->get_mytanks(&myTanks);
	/*if (myTanks[botIndex].callsign.find("blue"))
		color = "blue";
	else if (myTanks[botIndex].callsign.find("purple"))
		color = "purple";
	else if (myTanks[botIndex].callsign.find("green"))
		color = "green";
	else if (myTanks[botIndex].callsign.find("red"))
		color = "red";*/
	color = "green";
	//printf("\ncolor %s",myTanks[botIndex].callsign);
	myTeam->get_bases(&bases);
	colorBase = &bases[0];
	int i = 0;
	while (colorBase->color.compare(color)!=0)
	{
		colorBase = &bases[++i];
	}
	colorBase = new base_t();
	colorBase->color = bases[i].color;
	for (int j = 0; j < 4; j++)
	{
		colorBase->corner[j][0] = bases[i].corner[j][0];
		colorBase->corner[j][1] = bases[i].corner[j][1];
	}
	setBaseCenter(colorBase);

}

void PDAgent::Update(string color){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	vector <obstacle_t> myObst;
	vector <flag_t> allFlags;
	myTeam->get_mytanks(&myTanks);
	myTeam->get_obstacles(&myObst);
	myTeam->get_flags(&allFlags);
	flag_t goal = allFlags[0];
	Vector aForce;
	myTeam->shoot(botIndex);

	if (myTanks[botIndex].flag.compare("-") == 0){
		int i = 0;
		while (goal.color != color)//"red")
		{
			goal = allFlags[++i];
		}
		aForce = calcAttractiveForceToGoal(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), Vector(goal.pos[0], goal.pos[1]),
			0.5, 20, 1);
	}
	else {
		aForce = calcAttractiveForceToGoal(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), *baseCenter,
			0.5, 20, 1);
	}
	//end frobbing - f = a/dist + b
	//last param = range that obstacles affect bot.
	vector <Vector> rForces = calcRepulsiveForcesFromObstacles(
		Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), myObst, 40, 0, 40); //ranges were 100
	vector <Vector> tForces = calcTangentialForcesFromObstacles(
		Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), myObst, 100, 0, 50);

	newDirection += aForce;
	for each (Vector v in rForces)
	{
		newDirection += v;
	}
	for each(Vector v in tForces)
	{
		newDirection += v;
	}
	if (newDirection.x > 20.0)
		newDirection.x = 20.0;
	if (newDirection.y > 20.0)
		newDirection.y = 20.0;

	Vector avgVel = newDirection - oldVector;
	oldVector = newDirection;


	Vector sideVector;
	newDirection.perpendicular(&sideVector);
	double side = curVector->dot(sideVector);
	//finding the angle
	curVector->x = cos(myTanks[botIndex].angle);
	curVector->y = sin(myTanks[botIndex].angle);

	double dot = curVector->dot(newDirection);
	dot /= curVector->length()*newDirection.length();
	double angle = acos(dot);

	double avgAngVel = angle - oldAngle;
	oldAngle = angle;

	if (angle > 0.0000001 || angle < -0.0000001)
	{
		double newVel = (angle / (M_1_PI / 2.0)) + (100.0*avgAngVel);
		if (side < 0){
			myTeam->angvel(botIndex, newVel);
		}
		else{
			myTeam->angvel(botIndex, -newVel);
		}
	}
	double s = (newDirection.length() / 20);// +(-100 * avgVel.length())) / 20.0;
	if (s < 0.0)
		s = 0.0;
	myTeam->speed(botIndex, s);
}
boolean PDAgent::isInBase(base_t* team, flag_t* flag){
	double minx = 1000000;
	double miny = 1000000;
	double maxx = 0;
	double maxy = 0;
	for (int i = 0; i < 4; i++)
	{
		if (minx > team->corner[i][0])
			minx = team->corner[i][0];
		if (maxx < team->corner[i][0])
			maxx = team->corner[i][0];
		if (miny > team->corner[i][1])
			miny = team->corner[i][1];
		if (maxy > team->corner[i][1])
			maxy = team->corner[i][1];
	}
	if (flag->pos[0] > minx && flag->pos[0]<maxx && flag->pos[1]>miny && flag->pos[1] < maxy)
		return true;
	return false;
}

void PDAgent::setBaseCenter(base_t* base){
	//printf("Hi. . .");
	double minx = 9999.9;
	double miny = 9999.9;
	double maxx = -9999.9;
	double maxy = -9999.9;
	for (int i = 0; i < 4; i++)
	{
		if (minx > base->corner[i][0])
			minx = base->corner[i][0];
		if (maxx < base->corner[i][0])
			maxx = base->corner[i][0];
		if (miny > base->corner[i][1])
			miny = base->corner[i][1];
		if (maxy < base->corner[i][1])
			maxy = base->corner[i][1];
	}
	baseCenter = new Vector(maxx - 30, maxy - 30);
	//printf("baseCenter: %f %f", baseCenter->x, baseCenter->y);
}