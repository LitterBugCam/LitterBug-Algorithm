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


//in C++11 lambdas can't be templates :( too bad
template<class C, class T>
void inline divideArr(std::vector<C>& arr, const std::vector<T>& by)
{
    ALG_NS::transform (arr.cbegin(), arr.cend(), by.cbegin(), arr.begin(), std::divides<C>());
}

extern bool stop;
extern cv::Mat dir1, foreground1, dirsum, finalmap, bw, bw1;
extern std::vector< fullbits_int_t >  overlap_seg;
extern std::vector< fullbits_int_t > seg_processed;
extern bool debug;
void edge_segments(const cv::Mat &object_map, fullbits_int_t cc, fullbits_int_t rr, fullbits_int_t w, fullbits_int_t h, float &score, float &circularity);

#endif /* SCORING_H */

