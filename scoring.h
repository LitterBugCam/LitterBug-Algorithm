/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   scoring.h
 * Author: ilias
 *
 * Created on February 21, 2018, 2:34 PM
 */
#ifndef SCORING_H
#define SCORING_H
#include <cstdint>
#include <cassert>
#include <map>
#include <unordered_map>

#include "Litterheaders.h"

#define PI 3.14159265f
#define HALF_PI 1.570796f
#define QUATER_PI 0.785398

//in C++11 lambdas can't be templates :( too bad
template<class C, class T>
void inline divideArr(std::vector<C>& arr, const std::vector<T>& by)
{
    ALG_NS::transform (arr.cbegin(), arr.cend(), by.cbegin(), arr.begin(), std::divides<C>());
}


//well, instead using arrays and wasting a lot of memory, lets use map, however, it has limitations to 32-bit index

//this class works good with low amount of objects
template<class T>
class ZeroedMapArray
{
private:
    struct nohash
    {
        std::size_t operator()(uint64_t v)const noexcept
        {
            return v;
        }
    };
    using map_hash = typename std::conditional<sizeof(size_t) == sizeof(uint64_t), nohash, std::hash<uint64_t>>::type;
    std::unordered_map < uint64_t, T, map_hash> storage;

    uint64_t getKey(uint32_t x, uint32_t y) const
    {
        return (static_cast<uint64_t>(x) << 32) + y; //limits are because of that - packing 2 nums into 1
    }
public:
    using value_type = T;
    //mimic cv::Mat which uses at(y, x)
    T at(uint32_t y, uint32_t x) const
    {
        auto key = getKey(x, y);
        if (storage.count(key))
            return storage.at(key);
        return static_cast<T>(0);
    }

    T& at(uint32_t y, uint32_t x)
    {
        auto key = getKey(x, y);

        //just ensuring zero, however, never compilers set it zero by default...
        if (!storage.count(key))
            storage[key] = 0;

        return storage[key];
    }

    void clear()
    {
        storage.clear();
    }
};

//this will keep for experiments, so can just change between ZeroedMapArray / ZeroedArray in declaration only

//this class works good with big amount of objects
template <class T>
class ZeroedArray
{
private:
    cv::Mat storage;
public:
    using value_type = T;

    ZeroedArray(int size):
        storage(size, size, cv::DataType<T>::type, cv::Scalar(0)) {}

    ZeroedArray(int rows, int cols): //height, width
        storage(rows, cols, cv::DataType<T>::type, cv::Scalar(0)) {}

    ZeroedArray(cv::Size size):
        storage(std::move(size), cv::DataType<T>::type, cv::Scalar(0)) {}

    T at(uint32_t y, uint32_t x) const
    {
        assert(storage.type() == cv::DataType<T>::type);
        return storage.at<T>(y, x);
    }

    T& at(uint32_t y, uint32_t x)
    {
        assert(storage.type() == cv::DataType<T>::type);
        return storage.at<T>(y, x);
    }
    cv::Mat& getStorage()
    {
        return storage;
    }

    void clear()
    {
        storage *= 0;
    }

    T* ptr()
    {
        return storage.ptr<T>();
    }

    void resize(int rows, int cols)
    {
        storage.create(rows, cols, cv::DataType<T>::type);
    }
};

//just packing values to single struct, to minimize amount passed (same order as original source)
struct es_param_t
{
    fullbits_int_t cc;
    fullbits_int_t rr;
    fullbits_int_t w;
    fullbits_int_t h;
    float score;
    float circularity;
    es_param_t(fullbits_int_t cc, fullbits_int_t rr, fullbits_int_t w, fullbits_int_t h):
        cc(cc), rr(rr), w(w), h(h), score(0), circularity(0) {}
};

void edge_segments(const ZeroedArray<uint8_t> &object_map, const ZeroedArray<float> &dir1, const ZeroedArray<uint8_t>& canny, es_param_t& params);

#endif /* SCORING_H */

