CC := g++
C11 := -std=c++11
CFLAGS := -g -O0 
SRCS := Litter_detect.cpp  edge_grouping.cpp scoring.cpp
PROG := Litter_detect

OPENCV := `pkg-config opencv --cflags --libs`
LIBS := $(OPENCV)
PROFILING := -Wall -pg -no-pie
OPENMP := -fopenmp

$(PROG):$(SRCS)
	$(CC) $(C11) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
