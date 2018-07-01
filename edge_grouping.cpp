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

        if (ce.skip())
            continue;

        if (e != j && ce.positiongroup == 0)
        {
            if (minDistance(cj.origin, cj.endpoint, ce.origin, ce.endpoint) < 5)
            {
                if (cj.isCloseFrame(ce))
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

    //making surrounding boxes to calculated contours
    for (const auto & boxe : boxes)
    {
        bool found = false;
        const cv::Point2f box_center{boxe.x + boxe.width / 2.f, boxe.y + boxe.height / 2.f};

        for (auto & j : candidat)
        {
            j.positiongroup = 0;
            if (j == box_center)
            {
                j.update(box_center, boxe);
                j.activate();
                found = true;
                break;
            }
        }
        if (!found)
            candidat.emplace_back(box_center, boxe, newindex);
    }

    compteur = 1;
    for (size_t j = 0, sz = candidat.size(); j < sz; ++j)
    {
        if (candidat.at(j).skip())
            continue;

        grouping(j);
    }

    //fixme : this loop will have a bit different logic then original indexed for, because...original could crash on last frame erased....
    for (auto&  it : candidat)
    {
        it.lifetime++;

        if (it.deactivate())
            continue;

        if (it.skip())
        {
            const cv::Rect tmp{it.origin.x, it.origin.y, it.endpoint.x - it.origin.x, it.endpoint.y - it.origin.y};
            abandonnes.emplace_back(tmp, it.centre);

            //fixme: well, dunno, it seems logically for me to remove "skipped" candidate...
            //using "kill" recognizes woman as a whole bag at the end of movie + for 1 second recognizes bag in man's hands
            it.kill();
        }
    }
    cleanup(candidat);


    for (size_t j = 0, sz = candidat.size(); sz && (j < sz - 1); ++j)
    {
        const auto& cj = candidat.at(j);
        const fullbits_int_t label = candidat.at(j).positiongroup;
        cv::Rect obje;
        if (label != 0)
        {
            for (size_t e = j + 1, r = 0; e < sz; ++e, ++r)//r placed correct, original code does r = 0 prior 2nd loop
            {
                const auto& ce = candidat.at(e);
                if (label == ce.positiongroup)
                {
                    if (r == 0)
                        obje = cv::Rect(cj.origin, cj.endpoint) | cv::Rect(ce.origin, ce.endpoint); //union of 2 Rects
                    else
                        obje = cv::Rect(ce.origin, ce.endpoint) | obje;
                }
            }
            const cv::Point centre(obje.x + obje.width / 2, obje.y + obje.height / 2);
            bool found = false;

            for (auto & abandonne : abandonnes)
                if (abandonne == centre)
                {
                    found = true;
                    abandonne.tick(obje, centre);
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


