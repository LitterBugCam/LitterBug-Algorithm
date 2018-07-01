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
#include <algorithm>
extern cv::Mat map2_temp, threshed;

struct AO
{
    fullbits_int_t activeness{19}, lifetime{0}, abandoness{0}, objarea{0}, boxarea;
    bool update;
    cv::Point origin, endpoint, centre;

    AO() = delete;
    //original source had for 1st element update = true, and for any other new =false(actually missing),
    //it seems like bug for me, so let all be true for now
    AO(const cv::Rect& obje, const cv::Point& centre, bool update = true) :
        boxarea(obje.width * obje.height),
        update(update),
        origin{},
        endpoint{},
        centre()
    {
        tick(obje, centre);
    }

    void tick(const cv::Rect& obje, const cv::Point& centre)
    {
        this->centre = centre;
        origin   = {obje.x, obje.y};
        endpoint = {obje.x + obje.width, obje.y + obje.height};
        activeness = std::min(activeness + 1, static_cast<fullbits_int_t>(20));
    }

    bool operator==(const cv::Point& centre) const
    {
        return (abs(centre.x - this->centre.x) < 20 && abs(centre.y - this->centre.y) < 20);
    }
};

using AO_Collection = std::vector<AO>;

template <class T>
void cleanup(std::vector<T>& src)
{
    src.erase(std::remove_if(src.begin(), src.end(), [](T & val)->bool
    {
        return val.activeness < 1;
    }), src.end());
}

template <class T>
void next(std::vector<T>& src)
{
    ALG_NS::for_each(src.begin(), src.end(), [](T & i)
    {
        ++i.lifetime;
        --i.activeness;
    });
}

struct object
{
    bool proc{false};
    fullbits_int_t lifetime{0};
    fullbits_int_t apparition{0};
    cv::Point origin{};
    cv::Point centre{};
    cv::Point endpoint{};
    fullbits_int_t activeness{40};
    bool active{true};
    fullbits_int_t positiongroup{0};
    bool skip{false};
};

class objects
{
public:
    AO_Collection abandonnes{}, processed_objects{};
    fullbits_int_t compteur{0};
    std::vector<object> candidat{};

public:
    cv::Point::value_type minDistance(const cv::Point &p1, const cv::Point &p2, const cv::Point &q1, const cv::Point &q2);
    void grouping(const cv::Mat &image, size_t j);
    void extractObject(const cv::Mat &image, const cv::Mat &frame, fullbits_int_t i, const cv::Mat & map2);
    void reserve(long framesCount);

    objects() = default;
    objects(long framesCount);
};

#endif /* EDGE_GROUPING_H */



