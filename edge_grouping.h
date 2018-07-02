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
    fullbits_int_t video_frame_index; //video frame index I think
    fullbits_int_t activeness;
    fullbits_int_t initial_act;
    cv::Rect sourceBox{};
public:
    cv::Point origin{};
    cv::Point centre{};
    cv::Point endpoint{};

    object() = delete;
    object(const cv::Point& centre, const cv::Rect& boxe, fullbits_int_t video_frame_index, fullbits_int_t activeness = 40):
        video_frame_index(video_frame_index),
        activeness(activeness),
        initial_act(activeness)
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
        const auto ir = sourceBox & src.sourceBox;
        const auto ar = ir.area();
        return ar > sourceBox.area() * 0.9 || ar > src.sourceBox.area() * 0.9;
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

    struct es_param_t getScoreParams(fullbits_int_t rows, fullbits_int_t cols) const;
    bool isTooSmall(fullbits_int_t minsize) const
    {
        return std::abs(origin.y - endpoint.y) < 15 || std::abs(origin.x - endpoint.x) < minsize;
    }

    void extraLife()
    {
        activeness += initial_act;
    }
};
using obj_collection = std::vector<object>;
class objects
{
private:
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    fullbits_int_t obj_activeness;
public:
    obj_collection candidat{};
public:
    cv::Point::value_type minDistance(const cv::Point &p1, const cv::Point &p2, const cv::Point &q1, const cv::Point &q2);

    void populateObjects(const cv::Mat &image, fullbits_int_t newindex);
    void reserve(long framesCount);

    objects() = delete;
    objects(long framesCount, fullbits_int_t obj_activeness);
    void killOlds(); //kill all too olds
    void age(); //make each candidat older
};

#endif /* EDGE_GROUPING_H */



