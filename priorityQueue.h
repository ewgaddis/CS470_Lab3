#ifndef PRIORITY_QUEUE_H
#define PRIORITY_QUEUE_H

#include "team.h"
#include "geometry.h"

#include <vector>
#include <deque>

class PriorityQueue {
public:
	//vector<Vector> *pq;
	//vector<double> *costs;
	deque<int> *pq;
	deque<double> *costs;
	PriorityQueue();

		void push(int node,double priority);
		int pop();
		int at(int index);
		bool update(int node,double newPriority);
		int find(int node);
		bool empty();


private:
	
};

#endif