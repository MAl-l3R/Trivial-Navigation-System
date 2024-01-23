#include "dijkstra.h"

using namespace std;

/*
    Description:
        Used to sort the priority queue (min heap) from smallest to largest.
*/
class ComparableGreater {
 public:
	// Define the function call operator: operator()
	bool operator() (const PIPIL& lhs, const PIPIL& rhs) const {
		return (lhs.second.second > rhs.second.second);  // min heap
	}
};

/*
    Description:
        Computes least cost paths that start from a given vertex.
        Uses a binary heap to efficiently retrieve an unexplored
        vertex that has the minimum distance from the start vertex
        at every iteration.

        NOTE: PIL is an alias for "pair<int, long long>".
              PIPIL is an alias for "pair<int, PIL>".

	Arguments:
		graph(WDigraph&): an instance of the weighted directed graph class.
        startVertex(int): the root vertex of the search tree.
        tree(unordered_map): a search tree to construct the least cost path
                             from startVertex to some vertex.
*/
void dijkstra(const WDigraph& graph, int startVertex,
    unordered_map<int, PIL>& tree) {

    // each active fire is stored as (v, (u, d))
    // which implies that it is a fire started at u
    // currently burning the (u,v) edge
    // and will reach v at time d

    // A priority queue (min heap) to keep track of fires
    // (sorted from smallest to largest using the ComparableGreater funtion)
    priority_queue<PIPIL, std::vector<PIPIL>, ComparableGreater> fires;

    // At time 0, the startVertex burns, we set the predecesor of
    // startVertex to startVertex (as it is the first vertex)
    fires.push(PIPIL(startVertex, PIL(startVertex, 0)));

    // While there is an active fire
    while (fires.size() > 0) {
        // The fire that reaches its endpoint earliest is at the top of the
        // min heap, since min heap is already sorted from smallest to largest.
        auto earliestFire = fires.top();

        // v the current vertex to be explored.
        // u is the predessor of the vertex v.
        // d is the total cost/time to reach the current vertex.
        int v = earliestFire.first;
        int u = earliestFire.second.first;
        long long d = earliestFire.second.second;

        // Remove this fire.
        fires.pop();

        // If v is already "burned", there nothing to do.
        if (tree.find(v) != tree.end()) {
            continue;
        }

        // Record that 'v' is burned at time 'd' by a fire started from 'u'.
        tree[v] = PIL(u, d);

        // Now start fires from all edges exiting vertex 'v'.
        for (auto iter = graph.neighbours(v); iter != graph.endIterator(v);
            iter++) {
            int nbr = *iter;

            // 'v' catches on fire at time 'd' and the fire will reach 'nbr'
            // at time d + (length of v->nbr edge).
            long long t_burn = d + graph.getCost(v, nbr);
            fires.push(PIPIL(nbr, PIL(v, t_burn)));
        }
    }
    return;
}
