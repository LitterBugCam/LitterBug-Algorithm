#include "edge_grouping.h"
#include <limits>
#include "scoring.h"

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

    //making surrounding boxes to calculated contours
    if (!contours.empty() && !hierarchy.empty())
    {
        for (fullbits_int_t idx = 0; idx >= 0; idx = hierarchy.at(idx)[0])
        {
            const auto& c = contours.at(idx);
            if (cv::arcLength(c, false) > 20)
                boxes.push_back(cv::boundingRect(c));
        }
    }

    //if 2 boxes overlap more then 60% of 1 of the boxes - then join it

    for (size_t i = 0, back2 = 0, sz = boxes.size(); sz && i < sz - 1; ++i)
    {
        auto& b1 = boxes.at(i);
        for (size_t j = std::max(i + 1, back2); j < sz; ++j)
        {
            back2 = 0;
            const auto& b2 = boxes.at(j);
            const auto aj = (b1 & b2).area();
            if ( aj > 0.6 * b1.area() || aj > 0.6 * b2.area())
            {
                b1 = b1 | b2;
                sz -= 1;
                boxes.erase(boxes.begin() + j);
                i -= 1;
                back2 = j - 1;
                break;
            }
        }
    }


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
    std::sort(candidat.begin(), candidat.end(), [](object & o1, object & o2)
    {
        if (std::abs(o1.origin.x - o2.origin.x) < 3)
            return o1.origin.y < o2.origin.y;
        return o1.origin.x < o2.origin.x;
    });
    for (size_t j = 0, back2 = 0, sz = candidat.size(); sz && (j < sz - 1); ++j)
    {
        auto& cj = candidat.at(j);
        if (cj.active())
            for (size_t e = std::max(j + 1, back2); e < sz; ++e)//r placed correct, original code does r = 0 prior 2nd loop
            {
                auto& ce = candidat.at(e);
                if (ce.active())
                    if (cj.isCloseFrame(ce) && (minDistance(cj.origin, cj.endpoint, ce.origin, ce.endpoint) < 5 || cj.isFullyOverlap(ce)))
                    {
                        cj.join(ce);
                        ce.kill();
                        sz -= 1;
                        candidat.erase(candidat.begin() + e);
                        back2 = e - 1;
                        j -= 1;
                        break;
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

es_param_t object::getScoreParams(fullbits_int_t rows, fullbits_int_t cols) const
{
    const auto y = std::max(origin.y, 6);
    const auto x = std::max(origin.x, 6);

    const auto ey = std::min(endpoint.y, rows - 6);
    const auto ex = std::min(endpoint.x, cols  - 6);

    //dont you think it is suspicous - reverted w/h (and seems reverted again in edge_segment)
    return {y, x, ey, ex};
}
