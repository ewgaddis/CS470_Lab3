#ifndef GRAPH_H
#define GRAPH_H

#include "geometry.h"

#include <vector>

class Graph
{
private:
	int nextNodeID;

	std::vector<Vector> nodes;

	std::vector<std::vector<bool> > edges;

public:
	Graph();

	void addNode(const Vector &pos);
	void addEdge(int nodeID1, int nodeID2);
	void clear();

	int getNumberNodes() const { return nextNodeID; }

	const Vector & getNodePos(int id) const;

	bool edgeExist(int nodeID1, int nodeID2) const;
	void getNodesTo(int from, std::vector<int> *nodes) const;
};

#endif