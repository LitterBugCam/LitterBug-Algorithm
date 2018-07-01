
#ifndef LITTERHEADERS_H
#define LITTERHEADERS_H

#include<opencv2/opencv.hpp>
#include<vector>
#include<cstring>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cmath>
#include <algorithm>



//using fullbits_int_t = std::make_signed<size_t>::type;
using fullbits_int_t = int32_t;

#ifdef USING_OPENMP
    #include <parallel/algorithm>
    #define PROCESS_MSG
    #ifndef ALG_NS
        #define ALG_NS __gnu_parallel
    #endif

#endif

//fallback to std
#ifndef ALG_NS
    #define ALG_NS std
    #define PROCESS_MSG qApp->processEvents()
#endif

#endif
