#include <iostream>
#include <cassert>
#include <fstream>
#include <string>
#include <list>

#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>  // For c_str()

#include "wdigraph.h"
#include "digraph.h"
#include "dijkstra.h"

using namespace std;

/*
	Description:
		This struct is used to store the coordinates of a vertex in the graph.
*/
struct Point {
    long long lat;  // Latitude of the point
	long long lon;  // Longitude of the point
};

/*
    Description:
        Returns the Manhattan distance between the two given points.
	
	Arguments:
		pt1(Point&): a reference to one of the points/coordinates.
        pt2(Point&): a reference to the other point/coordinate.
	
    Returns:
		manDist(long long): Manhattan distance between the two given points.
*/
long long manhattan(const Point& pt1, const Point& pt2) {
    long long manDist = abs(pt1.lat - pt2.lat) + abs(pt1.lon - pt2.lon);
    return manDist;
}

/*
    Description:
        Returns the vertex/node that is closest to the given point.
	
	Arguments:
		point(Point&): a reference to the starting or ending point.
        points(unordered_map): map that maps each vertex ID to its coordinates.
	
    Returns:
		vertex(int): ID of the closest vertex/node to the given point.
*/
int findClosest(const Point& point, const unordered_map<int, Point>& points) {
  	int vertex = 0; 	  		  // Will store the nearest vertex's ID.
	int distance = 2147483647;  // Largest possible int value.
	for (auto it : points) {  // Get manhattan distance between given point and
		long long manDist = manhattan(it.second, point);  // all other vertices
		if (manDist <= distance) {  // Find the shortest manhattan distance.
			distance = manDist;
			vertex = it.first;   // Store the nearest vertex's ID.
		}
	}
  	return vertex;  	    // Return the nearest vertex's ID.
}

/*
    Description:
        Read Edmonton map data from the provided file and
		load it into the WDigraph object passed to this function.
		Stores vertex coordinates in Point struct and map each
		vertex identifier to its corresponding Point struct variable.

	Arguments:
		filename(string): the name of the file that describes a road network.
        graph(WDigraph&): an instance of the weighted directed graph class.
		points(unordered_map): map that maps each vertex ID to its coordinates.
*/
void readGraph(string filename, WDigraph& graph,
	unordered_map<int, Point>& points) {
	// Instantiate an object of input file stream to read from provided file
	ifstream file;
	file.open(filename);    // Open the file.

	while (!file.eof()) {  // Continue until EOF.
	    Point pt;  		 // Will hold the current coordinate.
		string input;   // Will hold the current input line from the file.
	    getline(file, input);  // Read a line from the file.

		// Get index of first comma.
	    int partition = input.find(',');

	    if (input[0] == 'V') {
			string ID, inLat, inLon;
			// Extract everything from after the first comma (i.e. after V).
			input = input.substr(partition + 1);
			// Get index of second comma.
			partition = input.find(',');

			// Extract the ID (starts at index 0 and is partition chars long).
			ID = input.substr(0, partition);
			// Convert string ID to integer ID.
			int v = stoi(ID);
			// Add the vertex, using its ID, to the graph.
			graph.addVertex(v);

			// Now, read the coordinates of the vertex...
			// Extract everything from after the second comma (i.e. after ID).
			input = input.substr(partition + 1);
			// Get index of third comma.
			partition = input.find(',');

			// Extract the latitude (starts at index 0 and is partition
			// characters long).
			inLat = input.substr(0, partition);
			double coord = stod(inLat);
			// Store the latitude of the vertex in 100,000-ths of a degree
			pt.lat = static_cast<long long>(coord*100000);

			// Extract the longitude (everything from after the third comma
			// (i.e. after latitude)).
			inLon = input.substr(partition + 1);
			coord = stod(inLon);
			// Store the longitude of the vertex in 100,000-ths of a degree
			pt.lon = static_cast<long long>(coord*100000);

			// Map the vertex, using its ID, to its coordinates.
			points[v] = pt;

        } else if (input[0] == 'E') {
            string vertex, vertices[2];   // Will hold the vertices.

			// Extract everything from after the first comma (i.e. after E).
			input = input.substr(partition + 1);
			for (int i = 0; i < 2; i ++) {
				// Get the index of the comma.
				partition = input.find(',');
				// Extract the ID (starts at index 0 and is partition
				// characters long).
				vertex = input.substr(0, partition);
				// Put the vertex safely aside for now.
				vertices[i] = vertex;
				// Extract everything from after the comma
				// (i.e. after current vertex ID).
				input = input.substr(partition + 1);
			}

            int u = stoi(vertices[0]);  // Convert string ID to integer ID.
            int v = stoi(vertices[1]);  // Convert string ID to integer ID.

			// Calculate the cost of this edge,
      		// i.e. the manhattan distance between the points.
      		long long cost = manhattan(points[u], points[v]);

			// Add the edge along with its cost.
      		graph.addEdge(u, v, cost);
    	}
  	}
  	file.close();  // Close the file.
  	return;
}

int create_and_open_fifo(const char * pname, int mode) {
    // creating a fifo special file in the current working directory
    // with read-write permissions for communication with the plotter
    // both proecsses must open the fifo before they can perform
    // read and write operations on it
    if (mkfifo(pname, 0666) == -1) {
        cout << "Unable to make a fifo. Ensure that this pipe does not exist already!" << endl;
        exit(-1);
    }

    // opening the fifo for read-only or write-only access
    // a file descriptor that refers to the open file description is
    // returned
    int fd = open(pname, mode);

    if (fd == -1) {
        cout << "Error: failed on opening named pipe." << endl;
        exit(-1);
    }

    return fd;
}

/* 
	Description:
		The main function calls readGraph() to read the map from the file
		"edmonton-roads-2.0.1.txt" and then responds to the client's request by
		first finding the closest vertices in the Edmonton’s road network to
		the start and end points according to the Manhattan distance, computing
		a shortest path along Edmonton streets between the two vertices found,
		and finally printing the found waypoints from first vertex to last back
*/
int main() {
    WDigraph graph;
    unordered_map<int, Point> points;

    const char *inpipe = "inpipe";
    const char *outpipe = "outpipe";

    // Open the two pipes
    int in = create_and_open_fifo(inpipe, O_RDONLY);
    cout << "inpipe opened..." << endl;
    int out = create_and_open_fifo(outpipe, O_WRONLY);
    cout << "outpipe opened..." << endl;

    // build the graph
    readGraph("server/edmonton-roads-2.0.1.txt", graph, points);

    // Continue until user quits the map (i.e. 'Q' is recieved).
    while (true) {
        Point sPoint, ePoint;
        char msg[200];  // Will hold input coordinates (start and end points).
        string temp = "", inputs = "";  // Will use these temporarily.
        long long journey[4];  // Will hold the input coordinates.
        int idx = 0;  // Index of journey array.

        read(in, msg, 200);  // Read the inputs using inpipe.

        // Look for the 'Q' command using the find() function of string.
        // If found, exit the loop and then clean up.
        for (int i = 0; i < 200; i++)
            inputs += msg[i];
        if (inputs.find('Q') % 200 == 0)
            break;

        // Parse through the input
        for (long unsigned int i = 0; i < inputs.size(); i++) {
            // If the current character is a space, we've read a coordinate.
            if (isspace(msg[i])) {
                // Store the coordinate in 100000ths of a degree in journey arr
                double coord = stod(temp);
                journey[idx] = static_cast<long long>(coord*100000);
                idx++;       // Increment journey array index.
                temp = "";  // Reset the temporary string for next coordinate.
                // If all four coordinates read, exit the loop.
                if (idx == 4)
                    break;

            // Otherwise, keep adding the characters to make the coordinate.
            } else {
                temp += msg[i];
            }
        }

        // Assign the starting and ending coordinates to sPoint and ePoint.
        sPoint.lat = journey[0];
        sPoint.lon = journey[1];
        ePoint.lat = journey[2];
        ePoint.lon = journey[3];

        // get the points closest to the two points we read
        int start = findClosest(sPoint, points), end = findClosest(ePoint, points);

        // run dijkstra's algorithm, this is the unoptimized version that
        // does not stop when the end is reached but it is still fast enough
        unordered_map<int, PIL> tree;
        dijkstra(graph, start, tree);

        // NOTE: in Part II you will use a different communication protocol than Part I
        // So edit the code below to implement this protocol

        string waypoints = "", outLat, outLon;
        // If path exists,
        if (tree.find(end) != tree.end()) {
            // read off the path by stepping back through the search tree
            list<int> path;
            while (end != start) {
                path.push_front(end);
                end = tree[end].first;
            }
            path.push_front(start);

            // output the path
            for (int v : path) {
                // Convert the coordinates to string and insert the decimal.
                outLat = to_string(points[v].lat);
                outLat.insert(2, ".");
                outLon = to_string(points[v].lon);
                outLon.insert(4, ".");
                // Create the waypoints in one string.
                waypoints = waypoints + outLat + ' ' + outLon + '\n';
            }
        }
        // Add the end of waypoints message 'E'.
        waypoints += "E\n";

        // Output the waypoints using outpipe.
        write(out, waypoints.c_str(), waypoints.size());
    }

    // Clean up
    close(in);
    close(out);
    unlink(inpipe);
    unlink(outpipe);

    return 0;
}
