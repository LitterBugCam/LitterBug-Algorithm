CC := g++
CFLAGS := -std=c++11 -g -O3 -march=native
FLAGSNG	:=	$(CFLAGS) -DNO_GUI

SRCS := Litter_detect.cpp  edge_grouping.cpp scoring.cpp
PROG := litter_detect

OPENCV := `pkg-config opencv --cflags --libs`
NOGUICV	:=	-lopencv_core -lopencv_videoio
LIBS := $(OPENCV)
#PROFILING := -Wall -pg -no-pie
#OPENMP := -fopenmp

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)

ngui:$(SRCS)
	$(CC) $(FLAGSNG) -o $(PROG) $(SRCS) $(NOGUICV)
