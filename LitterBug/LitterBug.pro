TEMPLATE = app
CONFIG += c++11
CONFIG -= app_bundle
CONFIG -= qt
#CONFIG += openmp

SOURCES += \
    ../edge_grouping.cpp \
    ../Litter_detect.cpp \
    ../scoring.cpp

HEADERS += \
    ../edge_grouping.h \
    ../Litterheaders.h \
    ../parameters.h \
    ../scoring.h

LIBS += -lopencv_core -lopencv_videoio -lopencv_imgproc -lopencv_highgui

QMAKE_CXXFLAGS +=  -Wctor-dtor-privacy -Werror=delete-non-virtual-dtor -fno-strict-aliasing
QMAKE_CXXFLAGS +=  -Werror=strict-aliasing -Wstrict-aliasing=2


CONFIG(debug, debug|release) {
     message( "Building the DEBUG Version" )
     #lets optimize for CPU on debug, for release - packager should do
     QMAKE_CXXFLAGS +=  -march=native -O0 -g
     DEFINES += _DEBUG
     DEFINES += DEBUG
     unix:!maxc:QMAKE_CXXFLAGS += -fsanitize=undefined -fsanitize=vptr
     unix:!maxc:LIBS += -lubsan
}
else {
    DEFINES += NDEBUG
    message( "Building the RELEASE Version" )
    #delegated to packager - didn't work easy, let it be here
    #QMAKE_CXXFLAGS += -O3 -march=native
    QMAKE_CXXFLAGS_RELEASE = -O3 -march=native
}


openmp {
message( "Building using OpenMP" )
DEFINES *= USING_OPENMP
QMAKE_CXXFLAGS *= -fopenmp
QMAKE_LFLAGS   *= -fopenmp
#uncomment to try global parallels
#DEFINES *= _GLIBCXX_PARALLEL
}
