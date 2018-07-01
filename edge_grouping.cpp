#include "edge_grouping.h"
#include <limits>

cv::Point::value_type objects::minDistance(const cv::Point& p1, const cv::Point& p2, const cv::Point& q1, const cv::Point& q2)
{
    using vt = cv::Point::value_type;
    vt mi = std::numeric_limits<vt>::max();

    // p1
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q1.x), std::abs<vt>(p1.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q1.x), std::abs<vt>(p1.y - q2.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q2.x), std::abs<vt>(p1.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q2.x), std::abs<vt>(p1.y - q2.y)));
    //p2
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q1.x), std::abs<vt>(p2.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q1.x), std::abs<vt>(p2.y - q2.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q2.x), std::abs<vt>(p2.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q2.x), std::abs<vt>(p2.y - q2.y)));
    //p1xp2y
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q1.x), std::abs<vt>(p2.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q1.x), std::abs<vt>(p2.y - q2.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q2.x), std::abs<vt>(p2.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p1.x - q2.x), std::abs<vt>(p2.y - q2.y)));
    //p1yp2x
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q1.x), std::abs<vt>(p1.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q1.x), std::abs<vt>(p1.y - q2.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q2.x), std::abs<vt>(p1.y - q1.y)));
    mi = std::min<vt>(mi, std::max<vt>(std::abs<vt>(p2.x - q2.x), std::abs<vt>(p1.y - q2.y)));
    return mi;
}

void objects::grouping(size_t j)
{
    auto& cj = candidat.at(j);
    for (size_t e = 0, sz = candidat.size(); e < sz; ++e)
    {
        auto& ce = candidat.at(e);
        if (e != j && ce.positiongroup == 0)
        {
            if (minDistance(cj.origin, cj.endpoint, ce.origin, ce.endpoint) < 5)
            {
                if (std::abs(cj.apparition - ce.apparition) < 50)
                {
                    if (cj.positiongroup == 0)
                    {
                        ce.positiongroup = compteur;
                        cj.positiongroup = compteur;
                        compteur++;
                        grouping(e);
                    }
                    else
                        if (cj.positiongroup > 0)
                        {
                            ce.positiongroup = cj.positiongroup;
                            grouping(e);
                        }
                }
            }
        }
    }
}

void objects::populateObjects(const cv::Mat &image, fullbits_int_t newindex)
{
    // cvtColor(map2, map2_temp, CV_GRAY2RGB);
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);

    if (!contours.empty() && !hierarchy.empty())
    {
        for (fullbits_int_t idx = 0; idx >= 0; idx = hierarchy.at(idx)[0])
        {
            const auto& c = contours.at(idx);
            if (cv::arcLength(c, false) > 20)
                boxes.push_back(cv::boundingRect(c));
        }
    }


    for (const auto & boxe : boxes)
    {
        bool found = false;
        const cv::Point2f sample{boxe.x + boxe.width / 2.f, boxe.y + boxe.height / 2.f};

        for (auto & j : candidat)
        {
            j.skip = false;
            j.positiongroup = 0;
            //~ if(blob->minx-candidat[j].origin.x<5 && blob->miny-candidat[j].origin.y<5 && blob->maxx-candidat[j].endpoint.x<5 && blob->maxy-candidat[j].endpoint.y<5 )
            if (std::abs(sample.x - j.centre.x) < 5 && std::abs(sample.y - j.centre.y) < 5)
            {

                j.centre.x = sample.x;
                j.centre.y = sample.y;
                j.origin.x = boxe.x;
                j.origin.y = boxe.y;
                j.endpoint.x = boxe.x + boxe.width;
                j.endpoint.y = boxe.y + boxe.height;

                j.activeness = 40;
                j.active = true;
                found = true;
                break;
            }

            //  rectangle(map2_temp, Rect(candidat[j].origin, candidat[j].endpoint), Scalar(0, 255, 255), 1);
            //   rectangle(threshed, Rect(candidat[j].origin, candidat[j].endpoint), Scalar(0, 255, 255), 1);

        }
        if (!found)
        {
            object obj;
            obj.positiongroup = 0;
            obj.centre.x = sample.x;
            obj.centre.y = sample.y;
            obj.origin.x = boxe.x;
            obj.origin.y = boxe.y;
            obj.endpoint.x = boxe.x + boxe.width;
            obj.endpoint.y = boxe.y + boxe.height;
            obj.lifetime = 0;
            obj.activeness = 40;
            obj.active = true;
            obj.apparition = newindex;
            candidat.push_back(obj);
        }
    }

    compteur = 1;
    for (size_t j = 0, sz = candidat.size(); j < sz; ++j)
    {
        if (candidat[j].skip)
            continue;
        grouping(j);
        //if (candidat[j].lifetime < 150);
        // rectangle(map2, Rect(candidat[j].origin, candidat[j].endpoint), Scalar(255, 0, 0));
    }

    //fixme : this loop will have a bit different logic then original indexed for, because...original could crash on last frame erased....
    for (auto&  it : candidat)
    {
        it.lifetime++;
        it.proc = false;
        if (--it.activeness < 1)
            continue;
        if (it.lifetime > 20 && it.positiongroup == 0)
        {
            //  cout << " efefefefef" << endl;
            it.skip = true;
            const cv::Rect tmp{it.origin.x, it.origin.y, it.endpoint.x - it.origin.x, it.endpoint.y - it.origin.y};
            abandonnes.emplace_back(tmp, it.centre);
        }
    }
    cleanup(candidat);

    for (size_t j = 0, sz = candidat.size(); j < sz; ++j)
    {
        fullbits_int_t label = candidat.at(j).positiongroup;
        cv::Rect obje;
        if (candidat.at(j).positiongroup != 0 && !candidat.at(j).proc)
            for (size_t e = 0, r = 0; e < sz; ++e, ++r)
            {
                if (e != j && label == candidat.at(e).positiongroup)//&& candidat[e].positiongroup!=0 )
                {
                    candidat.at(e).proc = true;
                    if (r == 0)
                        obje = cv::Rect(candidat.at(j).origin, candidat.at(j).endpoint) | cv::Rect(candidat.at(e).origin, candidat.at(e).endpoint);
                    else
                        obje = cv::Rect(candidat.at(e).origin, candidat[e].endpoint) | obje;
                }
            }
        const cv::Point centre(obje.x + obje.width / 2, obje.y + obje.height / 2);
        bool found = false;

        if (centre.x != 0 && centre.y != 0)
        {
            for (auto & abandonne : this->abandonnes)
                if (abandonne == centre)
                {
                    found = true;
                    abandonne.tick(obje, centre);

                    //~ break;
                }
            if (!found)
                abandonnes.emplace_back(obje, centre);
        }
    }

    next(abandonnes);
    cleanup(abandonnes);
}

void objects::reserve(long framesCount)
{
    abandonnes.reserve(framesCount * 50); //counting 50 objects per frame about...
    candidat.reserve(framesCount * 20);
}

objects::objects(long framesCount)
{
    reserve(framesCount);
}


