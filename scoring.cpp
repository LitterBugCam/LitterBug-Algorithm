#include <cmath>
#include "scoring.h"
#include <map>

bool stop;

cv::Mat dir1, foreground1, dirsum, finalmap, bw, bw1;

std::vector< fullbits_int_t >  overlap_seg;
std::vector< fullbits_int_t > seg_processed;
bool debug;

using segmap_t = short; //well, that is bad, on MC that can be 8 bits...and all opencb will be 8 bits as well >:

void edge_segments(const cv::Mat &object_map, fullbits_int_t cc, fullbits_int_t rr, fullbits_int_t w, fullbits_int_t h, float &score, float &circularity)
{
    const auto mhw = std::max(h, w); //not sure, i'm lost when to use h or w ...so just let it be square so all indexes are valid
    cv::Mat segmap = cv::Mat(mhw + 1, mhw + 1, cv::DataType<segmap_t>::type, cv::Scalar(0)); //(and +1 because of loops below)

    using namespace cv;
    size_t segcount = 1;
    for (fullbits_int_t r = rr; r < h; ++r)
        for (fullbits_int_t c = cc; c < w; ++c)
        {
            if (bw.at<uchar>(c, r) == 255 && object_map.at<uchar>(c, r) == 255)
            {

                float omin = 1000;
                fullbits_int_t cs = 0, rs = 0;
                bool neib = false;
                bool isolated = true;

                for (fullbits_int_t r0 = -1; r0 <= 1; ++r0)
                    for (fullbits_int_t c0 = -1; c0 <= 1; ++c0)
                    {

                        fullbits_int_t c1 = c + c0;
                        fullbits_int_t r1 = r + r0;

                        if (bw.at<uchar>(c1, r1) == 255)
                        {
                            isolated = false;
                            if (segmap.at<segmap_t>(c1, r1) != 0)
                            {

                                // test angle between p(c,r) and p1(c1,r1)

                                auto v = std::abs<float>(dir1.at<float>(c, r) - dir1.at<float>(c1, r1)) / PI;

                                if (v > 0.5) v = std::abs<float>(1 - v);

                                //test the min angle with neibghor
                                if (v < omin && dirsum.at<float>(c1, r1) < 0.5)
                                {

                                    cs = c1;
                                    rs = r1;
                                    neib = true;
                                    //~ segid=segmap.at<segmap_t>(c1,r1);
                                    omin = v;
                                }
                                // update dirsum
                            }
                        }
                    }
                if (neib == true)//&& omin!=1000)
                {
                    segmap.at<segmap_t>(c, r) = segmap.at<segmap_t>(cs, rs);
                    dirsum.at<float>(cs, rs) += omin;
                    dirsum.at<float>(c, r) = dirsum.at<float>(cs, rs);
                    finalmap.at<Vec3b>(c, r) = finalmap.at<Vec3b>(cs, rs);
                }//no segment exist in neigbhor, new segment is created starting from this point

                else
                    if (!isolated)
                    {
                        //~ cout<<"segments count "<<segcount<<endl;
                        //~ segmap1.at<edgep>(c,r).dirsum= new float(0);
                        segmap.at<segmap_t>(c, r) = segcount;
                        //~ RNG rng(12345);
                        //~ segmap1.at<edgep>(c,r).edgepoint= new Scalar(rand()&255, rand()&255, rand()&255);
                        finalmap.at<Vec3b>(c, r)[0] = rand() & 255;
                        finalmap.at<Vec3b>(c, r)[1] = rand() & 255;
                        finalmap.at<Vec3b>(c, r)[2] = rand() & 255;
                        segcount++;
                    }
            }
        }


    //prepare data for computing affinities


    overlap_seg.clear();
    std::vector<int>     meanX(segcount, 0), meanY(segcount, 0), meanNB(segcount, 0);
    std::vector<float>   meanOX(segcount, 0), meanOY(segcount, 0);

    for (fullbits_int_t r = rr; r < h; ++r)
        for (fullbits_int_t c = cc; c < w; ++c)
        {
            const auto& index = segmap.at<segmap_t>(c, r);
            if (index >= 0) //fixme: not sure, this check disallows 0th element in arrays divides, shouldn't it be >=0
            {
                meanX [index] += c;
                meanY [index] += r;
                meanOX[index] += cos(2 * dir1.at<float>(c, r));
                meanOY[index] += sin(2 * dir1.at<float>(c, r));
                meanNB[index] += 1;
            }
        }


    //starting from +1 because 0th is 0 = division by zero
    //this is ready to be parallized using openmp, and works faster then original loop with 1 core yet
    divideArr(meanX,  meanNB);
    divideArr(meanY,  meanNB);
    divideArr(meanOY, meanNB);
    divideArr(meanOX, meanNB);
    ALG_NS::transform (meanOX.cbegin(), meanOX.cend(), meanOY.cbegin(), meanOX.begin(), [](float ox, float oy)
    {
        return std::atan2(oy, ox) / 2;
    });
    const auto& meanO = meanOX;

    //compute segment convexity and degree of  parallelism with bounding box boundaries


    std::map<fullbits_int_t, std::map<fullbits_int_t, bool>> afinityidx;

    //compute inter-segments affinities
    for (fullbits_int_t r = rr + 2; r < h - 1; ++r)
        for (fullbits_int_t c = cc + 2; c < w - 1; ++c)
        {

            const fullbits_int_t s0 = segmap.at<segmap_t>(c, r);
            if (s0 <= 0)
                continue;

            for (fullbits_int_t rd = -2; rd <= 2; ++rd)
                for (fullbits_int_t cd = -2; cd <= 2; ++cd)
                {
                    const fullbits_int_t s1 = segmap.at<segmap_t>(c + cd, r + rd);
                    if (s1 <= s0)
                        continue;
                    if (afinityidx.count(s0) && afinityidx.at(s0).count(s1))
                        continue;

                    afinityidx[s0][s1] = true; //just adding any value
                    afinityidx[s1][s0] = true;
                }
        }
    score = pow(0.1, segcount - afinityidx.size());
    for (const auto& ap : afinityidx)
        score *= ap.second.size() / 2.f;

    /////////////////////////////////////////////////////:
    //~ cout<<"score "<<score<<endl;

    //~ if(score>0.001) waitKey(0);

    //segments parralelism with boundaries
    int64_t left = 0, right = 0, bot = 0, top = 0;


    const auto static topleft = [](fullbits_int_t a, fullbits_int_t b)
    {
        return a + (b - a) / 3;
    };

    const auto static botright = [](fullbits_int_t a, fullbits_int_t b)
    {
        return a + 2 * (b - a) / 3;
    };

    for (size_t i = 0; i < segcount; ++i)
    {
        const auto angle = std::abs<float>(sin(meanO[i]));

        if (std::abs<float>(angle - 1) < 0.5)
        {
            const int yeq1 =  rr + (h - rr) / 3;
            const int yeq2 =  rr + 2 * (h - rr) / 3;

            if (meanX[i] < topleft(cc, w) && meanY[i] > yeq1 && meanY[i] < yeq2)
                top += meanNB[i];

            if (meanX[i] > botright(cc, w) && meanY[i] >  yeq1  && meanY[i] < yeq2)
                bot += meanNB[i];
        }

        if (std::abs<float>(angle) < 0.5)
        {
            const int xeq1 = cc + 2 * (w - cc) / 3;
            const int xeq2 = cc + (w - cc) / 3;

            if (meanY[i] < topleft(rr, h) && meanX[i] < xeq1 && meanX[i] > xeq2)
                left += meanNB[i];
            if (meanY[i] > botright(rr, h) && meanX[i] < xeq1 && meanX[i] > xeq2)
                right += meanNB[i];
        }
    }

    if (left == 0) left = 1000000;
    if (top == 0) top = 1000000;
    if (bot == 0) bot = 1000000;
    if (right == 0) right = 1000000;

    //fixme: forced usage of 64 bits here to avoid overflows...but on 32 system it can be slower (still faster then doing floats)
    //1000000 ^ 2 = 1e12 and maximum for 64bits is 2^64 = 1.8e19
    const static auto spow = [](int64_t val)->uint64_t
    {
        return val * val;
    };



    float leftdiff = 1. / spow(h - rr - left), rightdiff = 1. / spow(h - rr - right),
          topdiff = 1. / spow(w - cc - top), botdiff = 1. / spow(w - cc - bot);

    circularity = (1000000000 * leftdiff * rightdiff * topdiff * botdiff); //pow(number,2);

    if (circularity > 10000000) circularity = 0;
}
