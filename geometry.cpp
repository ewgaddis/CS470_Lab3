#include "geometry.h"

#include <iostream>

using namespace std;

Vector operator * (double s, const Vector &v)
{
	return Vector(s * v.x, s * v.y);
}

std::ostream & operator << (std::ostream &out, const Vector &v)
{
	return out << "[" << v.x << ", " << v.y << "]";
}

double vectorDistance(const Vector &v1, const Vector &v2)
{
	double xDiff = v2.x - v1.x;
	double yDiff = v2.y - v1.y;

	return sqrt(xDiff * xDiff + yDiff * yDiff);
}

void closestPointOnLine(const Vector &endpoint0,
						const Vector &endpoint1,
						const Vector &p,
						Vector *q, Vector *v,
						double *dist)
{
	Vector line(endpoint1);
	line -= endpoint0;

	Vector u(p);
	u -= endpoint0;

	double d = line.dot(u);

	Vector closestPoint;

	if(d <= 0.0)
	{
		closestPoint = endpoint0;
	}
	else
	{
		double lineLenSq = line.lengthSq();

		if(d >= lineLenSq)
		{
			closestPoint = endpoint1;
		}
		else
		{
			double lineLen = sqrt(lineLenSq);
			d /= lineLen;

			Vector dir(line.x / lineLen, line.y / lineLen);

			closestPoint = sVector(dir.x * d, dir.y * d) + endpoint0;
		}
	}

	if(q)
	{
		*q = closestPoint;
	}

	if(v || dist)
	{
		Vector w(closestPoint.x - p.x, closestPoint.y - p.y);

		if(v)
		{
			*v = w;
		}

		if(dist)
		{
			*dist = w.length();
		}
	}
}

double cross(const Vector &u, const Vector &v)
{
	return u.x * v.y - u.y * v.x;
}

bool doLinesIntersect(const Vector &p0,
					  const Vector &p1,
					  const Vector &q0,
					  const Vector &q1)
{
	Vector line1(p1 - p0);
	Vector line2(q1 - q0);

	double c = cross(line1, line2);

	if(abs(c) < 0.00001)
	{
		return false;
	}

	double u = cross(q0 - p0, line1) / c;

	if(u < 0.0 || u > 1.0)
	{
		return false;
	}

	double v = cross(q0 - p0, line2) / c;

	if(v < 0.0 || v > 1.0)
	{
		return false;
	}

	return true;
}

bool isPointWithinObstacle(const Vector &p,
					       const Vector &c0,
					       const Vector &c1,
					       const Vector &c3)
{
	Vector min(c0);
	Vector max(c1.x, c3.y);

	if(p.x < min.x) return false;
	if(p.x > max.x) return false;

	if(p.y < min.y) return false;
	if(p.y > max.y) return false;

	return true;
}