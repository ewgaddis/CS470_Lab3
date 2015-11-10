#include "potentialFields.h"

#include "gnuplotter.h"

using namespace std;

void getMinDistV(const obstacle_t *obstacle,
				 const Vector &pos,
				 Vector *v, double *dist)
{
	Vector minV;
	double minDist = 999999.0;

	// Looks at each line of the obstacle defined by
	// its corner points
	for(int corner = 0; corner < obstacle->numCorners; ++corner)
	{
		// Gets the endpoints of the line
		int e0 = corner;
		int e1 = (corner + 1 == obstacle->numCorners ? 0 : corner + 1);

		Vector c0(obstacle->o_corner[e0][0],
					obstacle->o_corner[e0][1]);
		Vector c1(obstacle->o_corner[e1][0],
					obstacle->o_corner[e1][1]);

		// Gets the vector and distance from the closest
		// point on the line according to the given
		// position
		Vector vec;
		double distance;

		closestPointOnLine(c0, c1, pos, 0, &vec, &distance);

		// Only considers the shortest distance to the
		// obstacle
		if(distance < minDist)
		{
			minDist = distance;
			minV    = vec;
		}
	}

	*v    = minV;
	*dist = minDist;
}

Vector calcAttractiveForceToGoal(const Vector &pos, const Vector &goal,
								 double a, double max, double range)
{
	Vector force(goal.x - pos.x, goal.y - pos.y);

	force *= a;

	double forceLenSq = force.lengthSq();

	if(forceLenSq < range * range)
	{
		return Vector(0.0, 0.0);
	}
	else if(forceLenSq > max * max)
	{
		force *= max / sqrt(forceLenSq);
	}

	return force;
}

vector<Vector> calcRepulsiveForcesFromObstacles(const Vector &pos,
												const vector<obstacle_t> &obstacles,
												double a, double b, double range)
{
	// A list of repulsive forces
	vector<Vector> forces;

	// Goes through each obstacle
	vector<obstacle_t>::const_iterator itObstacle = obstacles.begin();
	while(itObstacle != obstacles.end())
	{
		Vector v;
		double k;

		// Gets the vector and distance to the obstacle
		getMinDistV(&(*itObstacle), pos, &v, &k);

		if(k > 0.0)
		{
			// Gets the force direction, which is the negative
			// direction of the vector to the "closest" line
			Vector forceDir(-v.x / k, -v.y / k);

			// Calculates the magnitude of the force
			double forceMag = 0.0;

			if(k < range)
			{
				// Force = 1 / k
				forceMag = a / k + b;
			}

			Vector force(forceDir * forceMag);

			if(force.lengthSq() > 0.0)
			{
				// Adds the force to the list
				forces.push_back(force);
			}
		}

		++itObstacle;
	}

	return forces;
}

vector<Vector> calcTangentialForcesFromObstacles(const Vector &pos,
												 const vector<obstacle_t> &obstacles,
												 double a, double b, double range)
{
	// A list of repulsive forces
	vector<Vector> forces;

	// Goes through each obstacle
	vector<obstacle_t>::const_iterator itObstacle = obstacles.begin();
	while(itObstacle != obstacles.end())
	{
		Vector v;
		double k;

		// Gets the vector and distance to the obstacle
		getMinDistV(&(*itObstacle), pos, &v, &k);

		if(k > 0.0)
		{
			// Gets the force direction, which is the
			// direction perpendicular to the vector to the
			// "closest" line
			Vector forceDir;
			Vector(v.x / k, v.y / k).perpendicular(&forceDir);

			// Calculates the magnitude of the force
			double forceMag = 0.0;

			if(k < range)
			{
				// Force = 1 / k
				forceMag = a / k + b;
			}

			Vector force(forceDir * forceMag);

			if(force.lengthSq() > 0.0)
			{
				// Adds the force to the list
				forces.push_back(force);
			}
		}

		++itObstacle;
	}

	return forces;
}

void drawPotentialField(GNUPlotter &plotter, void *ptr,
						int numSamples, int potentialField,
						double p1, double p2, double p3)
{
	int d = 800 / numSamples;

	for(int x = -400; x <= 400; x += d)
	{
		for(int y = -400; y <= 400; y += d)
		{
			Vector pos(x, y);
			vector<Vector> forces;

			switch(potentialField)
			{
			case 0:
				forces.push_back(calcAttractiveForceToGoal(pos, *(Vector *)ptr,
														   p1, p2, p3));
				break;

			case 1:
				forces = calcRepulsiveForcesFromObstacles(pos, *(vector<obstacle_t>*)ptr,
														  p1, p2, p3);
				break;

			case 2:
				forces = calcTangentialForcesFromObstacles(pos, *(vector<obstacle_t>*)ptr,
														   p1, p2, p3);
				break;
			}

			Vector netForce;

			vector<Vector>::iterator itForce = forces.begin();
			while(itForce != forces.end())
			{
				Vector force = (*itForce);

				netForce += force;
				++itForce;
			}

			if(netForce.lengthSq() < 1.0)
			{
				netForce.normalize();
			}

			plotter.drawArrow(pos.x, pos.y, netForce, 1);
		}
	}
}