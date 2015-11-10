#include "graph.h"

using namespace std;

Graph::Graph() : nextNodeID(0) {}

void Graph::addNode(const Vector &pos)
{
	nodes.push_back(pos);

	vector<vector<bool> >::iterator itFroms = edges.begin();
	while(itFroms != edges.end())
	{
		(*itFroms).push_back(false);
		++itFroms;
	}

	++nextNodeID;

	edges.push_back(vector<bool>(nextNodeID, false));
}

void Graph::addEdge(int nodeID1, int nodeID2)
{
	edges[nodeID1][nodeID2] = edges[nodeID2][nodeID1] = true;
}

void Graph::clear()
{
	edges.clear();
	nodes.clear();

	nextNodeID = 0;
}

const Vector & Graph::getNodePos(int id) const
{
	return nodes[id];
}

bool Graph::edgeExist(int nodeID1, int nodeID2) const
{
	return edges[nodeID1][nodeID2];
}

void Graph::getNodesTo(int from, std::vector<int> *nodes) const
{
	nodes->clear();

	for(int to = 0; to < nextNodeID; ++to)
	{
		if(edges[from][to])
		{
			nodes->push_back(to);
		}
	}
}