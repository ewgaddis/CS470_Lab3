#include "priorityQueue.h"

PriorityQueue::PriorityQueue(){
	pq = new deque<int>();//change to dequeue
	costs = new deque<double>();
}

void PriorityQueue::push(int node, double priority){
	bool inserted = false;
	for (int i = pq->size() - 1; i >= 0; i--){
		if (costs->at(i)>priority){
			if (i == pq->size() - 1){
				pq->push_back(node);
				costs->push_back(priority);
				inserted = true;
			}
			else{
				pq->insert(pq->begin() + i+1,node);
				costs->insert(costs->begin() + i + 1, priority);
				inserted = true;
			}
			break;
		}
	}
	if (!inserted){
		pq->push_front(node);
		costs->push_front(priority);
	}
}

int PriorityQueue::pop(){
	int back = pq->back();
	costs->pop_back();
	pq->pop_back();
	return back;
}

bool PriorityQueue::update(int node, double newPriority){
	//for updating node inside
	int index = find(node);
	if (index != -1 && newPriority < costs->at(index)){
		pq->erase(pq->begin() + index);
		costs->erase(costs->begin() + index);
		push(node, newPriority);
		return true;
	}
	return false;
}

int PriorityQueue::find(int node){
	for (int i = 0; i < pq->size(); i++){
		if (node == pq->at(i)){
			return i;
		}
	}
	return -1;
}

bool PriorityQueue::empty(){
	return pq->empty();
}

