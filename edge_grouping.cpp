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

    //grouping all closest which are not grouped yet
    for (bool once = true; once; once = !once)
    {
        for (size_t j = 0, sz = candidat.size(); sz && (j < sz - 1); ++j)
        {
            auto& cj = candidat.at(j);
            if (cj.active())
                for (size_t e = j + 1, r = 0; e < sz; ++e, ++r)//r placed correct, original code does r = 0 prior 2nd loop
                {
                    auto& ce = candidat.at(e);
                    if (ce.active())
                        if (cj.isCloseFrame(ce) && minDistance(cj.origin, cj.endpoint, ce.origin, ce.endpoint) < 5)
                        {
                            cj.join(ce);
                            ce.kill();
                            once = false;
                        }
                }
        }
    }


    ALG_NS::for_each(candidat.begin(), candidat.end(), [](object & c)
    {
        c.deactivate();
    });
    cleanup(candidat);
}

void objects::reserve(long framesCount)
{
    candidat.reserve(framesCount);
}

objects::objects(long framesCount)
{
    reserve(framesCount);
}


