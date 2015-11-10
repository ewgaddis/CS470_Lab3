#include "graph.h"
#include "graphAlgorithms.h"

#include <iostream>
#include <conio.h>

using namespace std;

void displayGraph(const Graph &graph)
{
	for(int i = 0; i < graph.getNumberNodes(); ++i)
	{
		cout << "Node #" << i << ": " << graph.getNodePos(i) << endl;
	}

	cout << "Adjacency Matrix" << endl;

	for(int i = 0; i < graph.getNumberNodes(); ++i)
	{
		cout << "[";

		for(int j = 0; j < graph.getNumberNodes(); ++j)
		{
			cout << (graph.edgeExist(i, j) ? 1 : 0);

			if(j != graph.getNumberNodes() - 1)
			{
				cout << " ";
			}
		}

		cout << "]" << endl;
	}
}

/*int main()
{
	Graph graph;

	Vector start(0.0f, 0.0f);
	Vector goal(10.0f, 10.0f);

	vector<obstacle_t> obstacles;
	obstacle_t obstacle;
	obstacle.numCorners = 4;
	obstacle.o_corner[0][0] = 2.0f;
	obstacle.o_corner[0][1] = 2.0f;
	obstacle.o_corner[1][0] = 6.0f;
	obstacle.o_corner[1][1] = 2.0f;
	obstacle.o_corner[2][0] = 6.0f;
	obstacle.o_corner[2][1] = 3.0f;
	obstacle.o_corner[3][0] = 2.0f;
	obstacle.o_corner[3][1] = 3.0f;
	obstacles.push_back(obstacle);
	obstacle.numCorners = 4;
	obstacle.o_corner[0][0] = 4.0f;
	obstacle.o_corner[0][1] = 8.0f;
	obstacle.o_corner[1][0] = 6.0f;
	obstacle.o_corner[1][1] = 8.0f;
	obstacle.o_corner[2][0] = 6.0f;
	obstacle.o_corner[2][1] = 4.0f;
	obstacle.o_corner[3][0] = 4.0f;
	obstacle.o_corner[3][1] = 4.0f;
	obstacles.push_back(obstacle);

	createVisibilityGraph(start, goal, obstacles, &graph);

	displayGraph(graph);

	while(!_kbhit());
	return 0;
}*/