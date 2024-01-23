#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include <iostream>
#include <unordered_map>
#include <vector>
#include <utility>		// std::pair
#include <queue>		// std::priority_queue
#include "wdigraph.h"

using namespace std;

// typedef creates an alias for the specified type
// PIL is the value type of our searchTree 
typedef pair<int, long long> PIL;

// PIPIL is used to insert a key-value pair in our searchTree
// if we declare a variable 'x' as follows:  PIPIL x;
// x.first gives the start vertex of the edge, 
// x.second.first gives the end vertex of the edge, 
// x.second.second gives the cost of the edge
typedef pair<int, PIL> PIPIL;

void dijkstra(const WDigraph& graph, int startVertex,
              unordered_map<int, PIL>& tree);

#endif
