CC = g++
CCFLAGS = -c -Wall -g -std=c++11
OBJS = digraph.o server.o dijkstra.o

# This will compile and link digraph.cpp, server.cpp, and dijkstra.cpp, and create server executable file.
server: digraph.o server.o dijkstra.o 
	$(CC) digraph.o server.o dijkstra.o -o server

# Create digraph object file using digraph.cpp and digraph.h.
digraph.o: digraph.cpp digraph.h
	$(CC) $(CCFLAGS) digraph.cpp

# Create server object file using server.cpp.
server.o: server.cpp
	$(CC) $(CCFLAGS) server.cpp

# Create dijkstra object file using dijkstra.cpp and dijkstra.h.
dijkstra.o: dijkstra.cpp dijkstra.h
	$(CC) $(CCFLAGS) dijkstra.cpp

# Remove all of the object files and the executable file.
clean:
	@rm $(OBJS) server
