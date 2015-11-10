#ifndef GRAPH_ALGORITHMS_H
#define GRAPH_ALGORITHMS_H

#include "graph.h"
#include "geometry.h"
#include "team.h"
#include "priorityQueue.h"

#include <vector>
#include <deque>
#include <stack>
#include <queue>

void createVisibilityGraph(const Vector &start,
						   const Vector &goal,
						   const std::vector<obstacle_t> &obstacles,
						   Graph *graph);

class GraphSearch
{
protected:
	const Graph *graph;

private:
	bool *visited;

	int *parent;

public:
	GraphSearch(const Graph &g) : graph(&g)
	{
		visited = new bool[g.getNumberNodes()];
		parent  = new int[g.getNumberNodes()];

		memset(visited, false, sizeof(bool) * g.getNumberNodes());
		memset(parent,  -1,    sizeof(int)  * g.getNumberNodes());
	}

	virtual ~GraphSearch()
	{
		delete[] visited;
		delete[] parent;
	}

	virtual bool search(int iterations) = 0;

	virtual void getFrontier(std::vector<int> *nodes) const = 0;

	void setVisited(int nodeID)      { visited[nodeID] = true; }
	bool isVisited(int nodeID) const { return visited[nodeID]; }
	const bool *getVisited() const   { return visited;         }

	void setParent(int nodeID, int parentID) { parent[nodeID] = parentID; }
	int  getParent(int nodeID) const         { return parent[nodeID];     }

	void getPath(std::deque<int> *path) const
	{
		path->clear();

		int node = 1;

		do
		{
			path->push_front(node);
			node = parent[node];
		}
		while(node != 0 && node != -1);

		if(node == 0)
		{
			path->push_front(0);
		}
	}
};

class DFSearch : public GraphSearch
{
private:
	stack<int> frontier;

public:
	DFSearch(const Graph &g);

	bool search(int iterations);

	void getFrontier(std::vector<int> *nodes) const;
};

class BFSearch : public GraphSearch
{
private:
	queue<int> frontier;
	bool contains(int node);

public:
	BFSearch(const Graph &g);

	bool search(int iterations);

	void getFrontier(std::vector<int> *nodes) const;
};

class ASearch : public GraphSearch
{
private:
	PriorityQueue frontier;//of ints
	int *cost;
	bool contains(int node);

public:
	ASearch(const Graph &g);

	bool search(int iterations);

	void getFrontier(std::vector<int> *nodes) const;
};

void drawGraphSearch(const Graph &graph,
					 GraphSearch *search,
					 int maxIterations,
					 const char *fileName,
					 const char *titleName);

#endif