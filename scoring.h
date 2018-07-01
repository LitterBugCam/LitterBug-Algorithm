/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   scoring.h
 * Author: ilias
 *
 * Created on February 21, 2018, 2:34 PM
 */
#ifndef SCORING_H
#define SCORING_H
#include <cstdint>
#include "Litterheaders.h"

#define PI 3.14159265f

using segmap_t = int16_t;


template<class T>
int getCvTypeFor1Channel()
{
    if (sizeof(T) == 8)
        return CV_64F;
    if (sizeof(T) == 1)
    {
        if (std::is_signed<T>::value)
            return CV_8SC1;
        else
            return CV_8UC1;
    }
    if (sizeof(T) == 2)
    {
        if (std::is_signed<T>::value)
            return CV_16SC1;
        else
            return CV_16UC1;
    }
    if (sizeof(T) == 4)
    {
        if (std::is_signed<T>::value)
            return CV_32S;
    }
    return CV_32F;
}

extern bool stop;
extern cv::Mat normm, dir1, foreground1, segmap, dirsum, finalmap, bw, bw1, object_map;
extern std::vector<std::vector<float>> afinity;
extern std::vector<std::vector<int>> afinityidx;
extern std::vector< fullbits_int_t >  overlap_seg;
extern std::vector< fullbits_int_t > seg_processed;
extern std::vector<float > segw;
extern std::vector<int> meanX, meanY, meanNB, segmag;
extern std::vector<float>   meanOX, meanOY, meanO;
extern bool debug;
void edge_segments(fullbits_int_t cc, fullbits_int_t rr, fullbits_int_t w, fullbits_int_t h, float &score, float &circularity);

#endif /* SCORING_H */

