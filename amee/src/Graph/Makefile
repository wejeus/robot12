OBJ	= Graph.o Node.o PathFinderAlgo.o
testG	= TestGraphAlgo.o ${OBJ}

CFLAGS	= -O2 -g -Wall

CC	= g++ 


all: compile


compile: ${testG}
	$(CC) $(CFLAGS) -o testG ${testG}

clean:
	\rm -f $(testG)

cleanall: clean
	\rm -f *~ testG

