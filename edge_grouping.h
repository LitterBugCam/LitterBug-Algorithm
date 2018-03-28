/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   edge_grouping.h
 * Author: ilias
 *
 * Created on February 21, 2018, 2:33 PM
 */

#ifndef EDGE_GROUPING_H
#define EDGE_GROUPING_H

#include "Litterheaders.h"

extern Mat map2_temp, threshed;

typedef struct {
    int activeness, lifetime, abandoness, objarea, boxarea;
    bool update;
    Point origin, endpoint, centre;
} AO;

typedef struct {
    bool proc;
    int lifetime;
    int apparition;
    Point origin;
    Point centre;
    Point endpoint;
    int activeness;
    bool active;
    int positiongroup;
    bool skip;

} object;

class objects {
public:
    vector<AO> abandonnes, processed_objects;

    int compteur;
    vector<object> candidat;
    int minDistance(Point p1, Point p2, Point q1, Point q2);
    void grouping(Mat &image, int j);
    void extractObject(Mat &image, Mat & frame, int i, Mat & map2);

};

#endif /* EDGE_GROUPING_H */



