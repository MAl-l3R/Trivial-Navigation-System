Description:
	A navigation system (like Google Maps) that allows the user to scroll around on a map of Edmonton, to select start and end points of a trip. The coordinates of these points are then sent to a route-finding server. This server uses Dijkstra’s Algorithm to compute the shortest path between the two selected points (or nearest points to them in the road network), and returns the route information (coordinates of the waypoints along the shortest path) to a plotter program. The plotter then displays the route as line segments overlaid on the original map by connecting the waypoints.

	Plotter Program Controls:
		• press W/S/A/D to move up/down/left/right the patch that is displayed in the window, respectively;
		• press R to remove all routes and selected points on the map;
		• press Q and E to respectively zoom in and out on the map, keeping the mouse cursor at its previous position;
		• click left mouse button to select current point as the start or end point of a trip;
		• drag left mouse button to scroll around on the map.


Included Files:
	1) Makefile
	2) README
	3) map/
		i) Contains PNG files used by the plotter.
	4) client/
		i) client.py
	5) server/
    	 	i) edmonton-roads-2.0.1.txt
    		ii) server.cpp
    	   	iii) dijkstra.cpp
    		iv) dijkstra.h
   	 	v) digraph.h
   		vi) digraph.cpp
       		vii) wdigraph.h                                                 
      		viii) Makefile


Running Instructions (Assuming Pygame is installed):
	With the terminal open in the main directory Trivial-Navigation-System/, run the command 'make clean', next run 'cd server && make server && cd ..', and then run 'make run'.