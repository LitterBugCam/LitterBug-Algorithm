CC = g++
CFLAGS = -g 
SRCS = Litter_detect.cpp  edge_grouping.cpp scoring.cpp
PROG = Litter_detect

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
