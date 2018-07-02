CC := g++
C11 := -std=c++11
CFLAGS := -g -O3 -march=native+fp -mfpu=auto -DNO_GUI

SRCS := Litter_detect.cpp  edge_grouping.cpp scoring.cpp
PROG := litter_detect

OPENCV := `pkg-config opencv --cflags --libs`
LIBS := $(OPENCV)
#PROFILING := -Wall -pg -no-pie
#OPENMP := -fopenmp

$(PROG):$(SRCS)
	$(CC) $(C11) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
