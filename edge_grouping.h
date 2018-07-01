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

    bool operator==(const AO& atu) const
    {
        return std::abs(origin.x - atu.origin.x) < 20 && std::abs(origin.y - atu.origin.y) < 20
               && std::abs(endpoint.x - atu.endpoint.x) < 20 && std::abs(endpoint.y - atu.endpoint.y) < 20;
    }

    bool active() const
    {
        return activeness > 0;
    }
};


using AO_Collection = std::vector<AO>;


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

public:
    fullbits_int_t lifetime{0};

    cv::Point origin{};
    cv::Point centre{};
    cv::Point endpoint{};

    fullbits_int_t positiongroup{0};

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
        origin.x = boxe.x;
        origin.y = boxe.y;
        endpoint.x = boxe.x + boxe.width;
        endpoint.y = boxe.y + boxe.height;
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

    bool deactivate() //returns true if object NOT active
    {
        --activeness;
        return !active();
    }

    bool skip() const
    {
        return lifetime > 20 && positiongroup == 0;
    }
};

class objects
{
public:
    AO_Collection abandonnes{};
    fullbits_int_t compteur{0};
    std::vector<object> candidat{};

public:
    cv::Point::value_type minDistance(const cv::Point &p1, const cv::Point &p2, const cv::Point &q1, const cv::Point &q2);

    void populateObjects(const cv::Mat &image, fullbits_int_t newindex);
    void reserve(long framesCount);

    objects() = default;
    objects(long framesCount);
private:
    void grouping(size_t j);
};

#endif /* EDGE_GROUPING_H */



