#include <cmath>
#include "scoring.h"
bool stop;
std::vector<int> segmag;

cv::Mat normm, dir1, foreground1, segmap, dirsum, finalmap, bw, bw1, object_map;
std::vector<std::vector<float>> afinity;
std::vector<std::vector<int>> afinityidx;
std::vector< fullbits_int_t >  overlap_seg;
std::vector< fullbits_int_t > seg_processed;
std::vector<float > segw;
bool debug;



void edge_segments(fullbits_int_t cc, fullbits_int_t rr, fullbits_int_t w, fullbits_int_t h, float &score, float &circularity)
{
    using namespace cv;
    size_t segcount = 1;
    for (fullbits_int_t r = rr; r < h; ++r)
        for (fullbits_int_t c = cc; c < w; ++c)
        {
            if (bw.at<uchar>(c, r) == 255 && object_map.at<uchar>(c, r) == 255)
            {

                float omin = 1000;
                fullbits_int_t cs, rs;
                bool neib = false;
                bool isolated = true;

                for (fullbits_int_t r0 = -1; r0 <= 1; r0++)
                    for (fullbits_int_t c0 = -1; c0 <= 1; c0++)
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
    afinityidx.clear();
    afinity.clear();
    afinityidx.resize(segcount);
    afinity.resize(segcount);

    //ensuring no allocations in deep loops below, 30 is multiply of all loops there
    for (size_t i = 0; i < segcount; ++i)
    {
        afinityidx.at(i).reserve(30);
        afinity.at(i).reserve(30);
    }

    //prepare data for computing affinities
    segw.assign(segcount, 0);
    segmag.assign(segcount, 0);

    overlap_seg.resize(0);
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

                const auto n = normm.at<float>(c, r) / 255.; //should be done where cardToPolar calculated, but here is faster
                //std::cout << " n = " << n << std::endl;
                segmag[index] += static_cast<segmap_t>(n); //Sobel gives float!!!!
                //~ cout<<"norm "<<(int)normm.at<uchar>(c,r)<<endl;
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




    //compute inter-segments affinities
    for (fullbits_int_t r = rr + 2; r < h - 1; r++)
        for (fullbits_int_t c = cc + 2; c < w - 1; c++)
        {

            fullbits_int_t s0 = segmap.at<segmap_t>(c, r);
            if (s0 <= 0)
                continue;

            for (fullbits_int_t rd = -2; rd <= 2; rd++)
                for (fullbits_int_t cd = -2; cd <= 2; cd++)
                {
                    fullbits_int_t s1 = segmap.at<segmap_t>(c + cd, r + rd);
                    if (s1 <= s0)
                        continue;

                    if (afinityidx.at(s0).cend() != std::find_if(afinityidx.at(s0).cbegin(), afinityidx.at(s0).cend(), [&s1](fullbits_int_t v)
                {
                    return v == s1;
                }))
                    continue;

                    float o = atan2(meanY[s0] - meanY[s1], meanX[s0] - meanX[s1]) + (PI / 2);
                    auto  a = std::abs<float>(cos(meanO[s0] - o) * cos(meanO[s1] - o));
                    a = pow(a, 2);
                    afinity[s0].push_back(a);
                    afinityidx[s0].push_back(s1);
                    afinity[s1].push_back(a);
                    afinityidx[s1].push_back(s0);
                }

        }



    float affine = 1;
    for (size_t i = 0, sz = afinityidx.size(); i < sz; ++i)
    {
        if (segw[i] >= 0.5) continue;
        if (afinityidx[i].size() == 0) affine = affine * 0.1;
        else affine = affine * ((float) afinityidx[i].size() / 2);


    }
    /////////////////////////////////////////////////////:
    //~ cout<<"score "<<score<<endl;

    //~ if(score>0.001) waitKey(0);

    //segments parralelism with boundaries
    float anglesum_left = 0, anglesum_top = 0, anglesum_bot = 0, anglesum_right = 0;
    fullbits_int_t left = 0, right = 0, bot = 0, top = 0;


    const auto static topleft = [](fullbits_int_t a, fullbits_int_t b)
    {
        return a + (b - a) / 3;
    };

    const auto static botright = [](fullbits_int_t a, fullbits_int_t b)
    {
        return a + 2 * (b - a) / 3;
    };

    for (size_t i = 0; i < segcount; i++)
    {
        if (segw[i] >= 1) continue;

        auto angle = std::abs<float>(sin(meanO[i]));
        // pow(angle, 2);

        const int yeq1 =  rr + (h - rr) / 3;
        const int yeq2 =  rr + 2 * (h - rr) / 3;

        //top boundary
        if (meanX[i] < topleft(cc, w) && meanY[i] > yeq1 && meanY[i] < yeq2)
        {

            anglesum_top += std::abs<float>(angle - 1) * meanNB[i];
            if (std::abs<float>(angle - 1) < 0.5)   top += meanNB[i];

        }
        //bot boundary
        if (meanX[i] > botright(cc, w) && meanY[i] >  yeq1  && meanY[i] < yeq2)
        {

            anglesum_bot += std::abs<float>(angle - 1) * meanNB[i];
            if (std::abs<float>(angle - 1) < 0.5)    bot += meanNB[i];

        }

        const int xeq1 = cc + 2 * (w - cc) / 3;
        const int xeq2 = cc + (w - cc) / 3;

        //left boundary
        if (meanY[i] < topleft(rr, h) && meanX[i] < xeq1 && meanX[i] > xeq2)
        {

            anglesum_left += std::abs<float>(angle * meanNB[i]);
            if (std::abs<float>(angle) < 0.5)  left += meanNB[i];

        }

        //right boundary
        if (meanY[i] > botright(rr, h) && meanX[i] < xeq1 && meanX[i] > xeq2)
        {

            anglesum_right += std::abs<float>(angle * meanNB[i]);
            if (std::abs<float>(angle) < 0.5)   right += meanNB[i];

        }

    }

    if (left == 0) left = 1000000;
    if (top == 0) top = 1000000;
    if (bot == 0) bot = 1000000;
    if (right == 0) right = 1000000;



    float leftdiff = 1 / pow((h - rr) - (float) left, 2), rightdiff = 1 / pow((h - rr) - (float) right, 2),
          topdiff = 1 / pow((w - cc) - (float) top, 2), botdiff = 1 / pow((w - cc) - (float) bot, 2);

    circularity = (1000000000 * leftdiff * rightdiff * topdiff * botdiff); //pow(number,2);

    if (circularity > 10000000) circularity = 0;

    score = affine;

}
