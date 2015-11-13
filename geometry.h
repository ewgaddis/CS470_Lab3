#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <math.h>
#include <ostream>

typedef struct sVector
{
	double x, y;

	sVector() : x(0.0), y(0.0) {}
	sVector(double _x, double _y) : x(_x), y(_y) {}
	sVector(const sVector &other) : x(other.x), y(other.y) {}

	sVector & operator = (const sVector &other)
	{
		x = other.x;
		y = other.y;

		return *this;
	}

	sVector operator + (const sVector &other) const
	{
		return sVector(x + other.x, y + other.y);
	}

	sVector operator - (const sVector &other) const
	{
		return sVector(x - other.x, y - other.y);
	}

	sVector operator * (double s) const
	{
		return sVector(x * s, y * s);
	}

	sVector & operator += (const sVector &other)
	{
		x += other.x;
		y += other.y;

		return *this;
	}

	sVector & operator -= (const sVector &other)
	{
		x -= other.x;
		y -= other.y;

		return *this;
	}

	sVector & operator *= (double s)
	{
		x *= s;
		y *= s;

		return *this;
	}

	sVector operator - () const
	{
		return sVector(-x, -y);
	}

	bool operator == (const sVector &other) const
	{
		return (x == other.x && y == other.y);
	}

	bool operator != (const sVector &other) const
	{
		return (x != other.x || y != other.y);
	}

	double lengthSq() const
	{
		return x * x + y * y;
	}

	double length() const
	{
		if(x == 0.0 && y == 0.0)
		{
			return 0.0;
		}

		return sqrt(lengthSq());
	}

	void normalize()
	{
		if(x == 0.0 && y == 0.0)
		{
			return;
		}

		double len = length();

		x /= len;
		y /= len;
	}

	void perpendicular(sVector *v) const
	{
		v->x = -y;
		v->y = x;
	}

	double dot(const sVector &v) const
	{
		return x * v.x + y * v.y;
	}
} Vector;

Vector operator * (double s, const Vector &v);
std::ostream & operator << (std::ostream &out, const Vector &v);

double vectorDistance(const Vector &v1, const Vector &v2);

typedef struct sEdge
{
	Vector e0;
	Vector e1;

	sEdge() : e0(), e1() {}
	sEdge(const Vector &end0, const Vector &end1) : e0(end0), e1(end1) {}
	sEdge(const sEdge &other) : e0(other.e0), e1(other.e1) {}
} Edge;

void closestPointOnLine(const Vector &endpoint0,
						const Vector &endpoint1,
						const Vector &p,
						Vector *q, Vector *v,
						double *dist);

bool doLinesIntersect(const Vector &p0,
					  const Vector &p1,
					  const Vector &q0,
					  const Vector &q1);

bool isPointWithinObstacle(const Vector &p,
					       const Vector &c0,
					       const Vector &c1,
					       const Vector &c3);

#endif