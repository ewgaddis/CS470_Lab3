#include "scoutAgent.h"
#include "geometry.h"
#include <math.h>

ScoutAgent::ScoutAgent(BZRC* team, int index,string area){
	myTeam = team;
	deque<int> p;

	deque<int>::iterator itNode = p.begin();// +1;

	if (area == "upper"){
		double x = 400;
		double y = 0;
		//while (itNode != p.end())
		while (y <=400)
		{
			Vector pos1 = Vector(x, y);
			path.push_back(pos1);
			//++itNode;
			if (x == 400)
				x = -400;
			else
				x = 400;
			y += 100;
		}
	}
	
	botIndex = index;
	curVector = new Vector();
	oldVector = Vector();
	oldLoc = Vector();
	vector <tank_t> myTanks;
	oldAngle = 0.0;
	maxDist = 10;
	time = 0;
	maxtime = 30;
}

void ScoutAgent::Update(){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	vector <obstacle_t> myObst;
	vector <flag_t> allFlags;
	myTeam->get_mytanks(&myTanks);
	//flag_t goal = allFlags[0];
	Vector aForce;
	myTeam->shoot(botIndex);
	Vector curGoal;

	if (time >= maxtime)
	{
		time = 0;
		Vector* direction = new Vector(myTanks[botIndex].velocity[0], myTanks[botIndex].velocity[1]);
		Vector* newDir = new Vector();
		direction->perpendicular(newDir);
		//	P3 = (-v.y, v.x) / Sqrt(v.x ^ 2 + v.y ^ 2) * h
		Vector top = Vector(-myTanks[botIndex].velocity[1], myTanks[botIndex].velocity[0]);
		double scalar = sqrt(myTanks[botIndex].velocity[0] * myTanks[botIndex].velocity[0] +
			myTanks[botIndex].velocity[1] * myTanks[botIndex].velocity[1]) * 50;
		Vector newGoal = Vector(top.x / scalar, top.y / scalar);
		printf("\n x=%f y=%f", newGoal.x, newGoal.y);
		path.push_front(newGoal);
		//Vector* newGoal = new Vector(myTanks[botIndex].pos[0] + newDir->x, myTanks[botIndex].pos[1] + newDir->y);
		//get perpendicular angle to direction and move 50 in that direction.
		//push back on old goal and add the above new one in front of that one.
	}
	//if (myTanks[botIndex].flag.compare("-") == 0){
		int i = 0;
		/*while (goal.color != color)//"red")
		{
		goal = allFlags[++i];
		}*/
		//curGoal = path.front();
		if (path.size() != 0)
			curGoal = path.front();
		else
			curGoal = Vector(0, 0);
		if (isCloseToGoal(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), curGoal)){
			if (!path.empty())
				path.pop_front();
			if (path.size() != 0)
				curGoal = path.front();
			else
				curGoal = Vector(0, 0);
		}

		aForce = calcAttractiveForceToGoal(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), curGoal, 0.5, 20, 1);//Vector(goal.pos[0], goal.pos[1]),
		//0.5, 20, 1);
	//}
	//end frobbing - f = a/dist + b
	//last param = range that obstacles affect bot.
	/*vector <Vector> rForces = calcRepulsiveForcesFromObstacles(
		Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), myObst, 10, 0, 10); //range was 40 a was 40
	vector <Vector> tForces = calcTangentialForcesFromObstacles(
		Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), myObst, 10, 0, 10);//range was 50 a was 100
		*/

	newDirection += aForce;
	/*for each (Vector v in rForces)
	{
		newDirection += v;
	}
	for each(Vector v in tForces)
	{
		newDirection += v;
	}*/
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
	double s = (newDirection.length() / 20)*dot;// +(-100 * avgVel.length())) / 20.0;
	if (s < 0.0)
		s = 0.0;
	myTeam->speed(botIndex, s);
	//myTeam->accelx(botIndex,)
	//change this to distance from last location :P
	//d=sqrt((x2-x1)^2+(y2-y1)^2)
	double dist = sqrt((myTanks[botIndex].pos[0] - oldLoc.x)*(myTanks[botIndex].pos[0] - oldLoc.x) + 
		(myTanks[botIndex].pos[1] - oldLoc.y)*(myTanks[botIndex].pos[1] - oldLoc.y));
	if (dist < 20 && dist > -20){ //if bot's current speed, not the assigned speed == 0
		time++;
		printf("\nTic");
	}
}

boolean ScoutAgent::isCloseToGoal(Vector location, Vector goal){
	double distance = sqrt(((location.x - goal.x)*(location.x - goal.x)) + ((location.y - goal.y)*(location.y - goal.y)));
	if (distance < maxDist)
		return true;
	return false;
}