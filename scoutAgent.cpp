#include "scoutAgent.h"
#include "geometry.h"
#include <math.h>
using namespace std;
ScoutAgent::ScoutAgent(BZRC* team, int index,string area){
	myTeam = team;
	search = NULL;
	graph = NULL;
	numObstacles = 0;
	deque<int> p;

	deque<int>::iterator itNode = p.begin();// +1;

	/*if (area == "upper"){
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
	}*/
	
	botIndex = index;
	curVector = new Vector();
	oldVector = Vector();
	oldLoc = Vector();
	vector <tank_t> myTanks;
	oldAngle = 0.0;
	maxDist = 10; // 10;
	time = 0;
	maxtime = 100;
}

void ScoutAgent::Update(vector <obstacle_t> obstacles, OCCGrid * grid){
	Vector newDirection = Vector();
	vector <tank_t> myTanks;
	vector <obstacle_t> myObst = obstacles;
	vector <flag_t> allFlags;
	myTeam->get_mytanks(&myTanks);
	//flag_t goal = allFlags[0];
	Vector aForce;
	//myTeam->shoot(botIndex);
	Vector curGoal;
	//get current path
	if (graph == NULL){
		graph = new Graph();
		createVisibilityGraph(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), Vector(-400, 400), obstacles, graph);
		recalculatePath(curGoal, myTanks, myObst);
	}
		
	if(numObstacles != myObst.size()){
		recalculatePath(curGoal,myTanks,myObst);
	}

	//1st dimention = j = rows, 2nd is i = column,  coord bottom left is 0 top right = 800 800 i-400, j-400 = position.
	//if (myTanks[botIndex].flag.compare("-") == 0){
		int i = 0;
		/*while (goal.color != color)//"red")
		{
		goal = allFlags[++i];
		}*/
		//curGoal = path.front();
		if (path.size() != 0)
			curGoal = path.front();
		else{
			path.push_back(Vector(-400, -400));
		}
		double ** g = grid->getGrid();
		boolean curIn = isInObstacle(curGoal, myObst);
		boolean finalIn = isInObstacle(path.back(), myObst);

		if (maxtime>=time || curIn || finalIn || isCloseToGoal(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), curGoal)){
			if (finalIn || curIn || maxtime<=time){
				path.clear();
			}
			if (!path.empty() && maxtime >= time)
				path.pop_front();
			if (path.size() != 0 && maxtime >= time)
				curGoal = path.front();
			else{
				//curGoal = Vector(0, 0);
				int i = 0;
				int j = 0;
				boolean found = false;
				for (j = 0; j < grid->getGridSize(); j++){
					for (i = 0; i < grid->getGridSize(); i++){
						//printf("\nHmm2: %f", g[j][i]);
						if (g[j][i] < 0.9 && g[j][i] > 0.1){
							int newI = i-400;
							int newJ = j-400;
							if (newI <= -390)
								newI += 10;
							else if (newI >= 390)
								newI -= 10;
							if (newJ <= -390)
								newJ += 10;
							else if (newJ>=390)
								newJ -= 10;
							//path.push_back(Vector(newI, newJ));
							found = true;
							curGoal = Vector(newI, newJ);//path.front();
							recalculatePath(curGoal, myTanks, myObst);
							printf("\nnew goal2: %d %d", newI, newJ);
							break;
						}
					}
					if (found)
						break;
				}
			}
		}
		printf("\nCurGoal: %f %f", curGoal.x, curGoal.y);
		printf("\nfinalGoal: %f %f", path.back().x, path.back().y);
		aForce = calcAttractiveForceToGoal(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), curGoal, 0.5, 20, 1);//Vector(goal.pos[0], goal.pos[1]),
		//0.5, 20, 1);
	//}
	//end frobbing - f = a/dist + b
	//last param = range that obstacles affect bot.
	vector <Vector> rForces = calcRepulsiveForcesFromObstacles(
		Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), myObst, 10, 0, 10); //range was 40 a was 40
	vector <Vector> tForces = calcTangentialForcesFromObstacles(
		Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), myObst, 10, 0, 10);//range was 50 a was 100
	

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
	double s = (newDirection.length() / 20)*dot;// +(-100 * avgVel.length())) / 20.0;
	if (s < 0.0)
		s = 0.0;
	if (s > 1.0)
		s = 1.0;
	printf("\nSpeed: %f", s);
	myTeam->speed(botIndex, s);
	time++;
	//myTeam->accelx(botIndex,)
	//change this to distance from last location :P
	//d=sqrt((x2-x1)^2+(y2-y1)^2)
	/*double dist = sqrt((myTanks[botIndex].pos[0] - oldLoc.x)*(myTanks[botIndex].pos[0] - oldLoc.x) + 
		(myTanks[botIndex].pos[1] - oldLoc.y)*(myTanks[botIndex].pos[1] - oldLoc.y));
	if (dist < 20 && dist > -20){ //if bot's current speed, not the assigned speed == 0
		time++;
		printf("\nTic");
	}*/
}

boolean ScoutAgent::isCloseToGoal(Vector location, Vector goal){
	double distance = sqrt(((location.x - goal.x)*(location.x - goal.x)) + ((location.y - goal.y)*(location.y - goal.y)));
	if (distance < maxDist)
		return true;
	return false;
}

void ScoutAgent::recalculatePath(Vector curGoal, vector<tank_t>myTanks, vector<obstacle_t> obstacles){
	/*if (search!=NULL)
		delete search;
	if (graph!=NULL)
		delete graph;*/
	Vector goal = Vector(curGoal);
	createVisibilityGraph(Vector(myTanks[botIndex].pos[0], myTanks[botIndex].pos[1]), goal, obstacles, graph);
	path.clear();
	search = new ASearch(*graph);
	search->search(100);
	deque<int> p;
	search->getPath(&p);

	deque<int>::iterator itNode = p.begin();// +1;
	while (itNode != p.end())
	{
		Vector pos1 = graph->getNodePos(*(itNode));// -1));
		//Vector pos2 = g.getNodePos(*(itNode));
		path.push_back(pos1);
		//drawArrow(pos1, pos2, 1);
		++itNode;
	}
}

boolean ScoutAgent::isInObstacle(Vector pos, vector<obstacle_t> obstacles){
	
	double minx = 9999.9;
	double miny = 9999.9;
	double maxx = -9999.9;
	double maxy = -9999.9;
	//obstacles.at(0).numCorners;
	for (int o = 0; o < obstacles.size(); o++)
	{
		obstacle_t obstacle = obstacles.at(o);
		for (int i = 0; i < obstacle.numCorners; i++)
		{
			if (pos.x == obstacle.o_corner[i][0] && pos.y == obstacle.o_corner[i][1])
				return false;
			if (minx > obstacle.o_corner[i][0])
				minx = obstacle.o_corner[i][0];
			if (maxx < obstacle.o_corner[i][0])
				maxx = obstacle.o_corner[i][0];
			if (miny > obstacle.o_corner[i][1])
				miny = obstacle.o_corner[i][1];
			if (maxy < obstacle.o_corner[i][1])
				maxy = obstacle.o_corner[i][1];
		}
		if (pos.x <= maxx && pos.x >= minx && pos.y <= maxy && pos.y >= miny){
			printf("\nhello? %f %f", pos.x, pos.y);
			return true;
		}
	}
	//baseCenter = new Vector(maxx - 30, maxy - 30);

	return false;
}