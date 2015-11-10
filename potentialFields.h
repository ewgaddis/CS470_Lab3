#ifndef POTENTIAL_FIELDS_H
#define POTENTIAL_FIELDS_H

#include "team.h"
#include "geometry.h"

#include <vector>

class GNUPlotter;

Vector calcAttractiveForceToGoal(const Vector &pos, const Vector &goal,
								 double a, double max, double range);

std::vector<Vector> calcRepulsiveForcesFromObstacles(const Vector &pos,
													 const std::vector<obstacle_t> &obstacles,
													 double a, double b, double range);

std::vector<Vector> calcTangentialForcesFromObstacles(const Vector &pos,
													  const std::vector<obstacle_t> &obstacles,
													  double a, double b, double range);

void drawPotentialField(GNUPlotter &plotter, void *ptr,
						int numSamples, int potentialField,
						double p1, double p2, double p3);

#endif