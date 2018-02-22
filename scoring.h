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
#include "Litterheaders.h"

#define PI 3.14159265f

extern bool stop;
extern Mat normm, dir1, foreground1, segmap, dirsum, finalmap, bw, bw1, object_map;
extern vector<vector<float> > afinity;
extern vector<vector<int> > afinityidx;
extern vector< int >  overlap_seg;
extern vector< int > seg_processed;
extern vector<float > segw;
extern vector<int> meanX, meanY, meanNB, segmag;
extern vector<float>   meanOX, meanOY, meanO;
extern bool debug;
void edge_segments(int cc, int rr, int w, int h, float &score, float &circularity);

#endif /* SCORING_H */

