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


template <class T>
void sortX(std::vector<T>& src)
{
    std::sort(src.begin(), src.end(), [](T & a, T & b)
    {
        return a.origin.x < b.origin.x;
    });
}

template <class T>
void cleanup(std::vector<T>& src)
{
    src.erase(std::remove_if(src.begin(), src.end(), [](T & val)->bool
    {
        return !val.active();
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
private:
    fullbits_int_t activeness{40};
    fullbits_int_t video_frame_index; //video frame index I think
    cv::Rect sourceBox{};
public:
    cv::Point origin{};
    cv::Point centre{};
    cv::Point endpoint{};

    object() = delete;
    object(const cv::Point& centre, const cv::Rect& boxe, fullbits_int_t video_frame_index):
        video_frame_index(video_frame_index)
    {
        update(centre, boxe);
    }

    bool operator == (const cv::Point& box_center) const
    {
        return std::abs(box_center.x - centre.x) < 5 && std::abs(box_center.y - centre.y) < 5;
    }

    bool isCloseFrame(const object& o) const
    {
        return std::abs(video_frame_index - o.video_frame_index) < 50;
    }

    void update(const cv::Point& centre, const cv::Rect& boxe)
    {
        this->centre = centre;
        sourceBox = boxe;
        origin.x = boxe.x;
        origin.y = boxe.y;
        endpoint.x = boxe.x + boxe.width;
        endpoint.y = boxe.y + boxe.height;
    }

    void join(const object& src)
    {
        cv::Point centre((origin.x + src.origin.x) / 2, (origin.y + src.origin.y) / 2);
        update(centre, sourceBox | src.sourceBox);
        activeness = std::max(activeness, src.activeness) + 1;
    }

    bool isFullyOverlap(const object& src) const
    {
        //checks if this contais src or src contains this completely
        const cv::Rect r1(origin, endpoint);
        const cv::Rect r2(src.origin, src.endpoint);
        const auto ir = r1 & r2;
        const auto ar = ir.area();
        return ar > r1.area() * 0.9 || ar > r2.area() * 0.9;
    }

    bool active() const
    {
        return activeness > 0;
    }

    bool activate()
    {
        ++activeness;
        return active();
    }

    bool deactivate()
    {
        --activeness;
        return active();
    }

    void kill()
    {
        activeness = -100;
    }
};
using obj_collection = std::vector<object>;
class objects
{
public:
    obj_collection candidat{};
public:
    cv::Point::value_type minDistance(const cv::Point &p1, const cv::Point &p2, const cv::Point &q1, const cv::Point &q2);

    void populateObjects(const cv::Mat &image, fullbits_int_t newindex);
    void reserve(long framesCount);

    objects() = default;
    objects(long framesCount);
};

#endif /* EDGE_GROUPING_H */



