EXEC = server/server
OBJS = server/digraph.o server/server.o server/dijkstra.o
PIPE = inpipe outpipe

# Runs the server program in a new terminal and then runs the client program.
run:
	@$(MAKE) run-server &
	sleep 1
	@$(MAKE) run-client &

# Runs the server program.
run-server:
	./server/server

# Runs the client program.
run-client:
	python3 client/client.py

# Removes all executables, objects, and named pipes.
clean:
	@rm -f $(EXEC) $(OBJS) $(PIPE)
