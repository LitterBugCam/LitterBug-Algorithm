#include <string>
#include <type_traits>
#include <algorithm>
#include <functional>
#include <cctype>
#include <locale>
#include <cstdint>
#include "Litterheaders.h"

template <class T>
T str2num(const std::string& str)
{
    if (std::is_floating_point<T>())
        return static_cast<T>(std::stod(str));
    return static_cast<T>(std::stoi(str));
}

template <class T>
void setParam(const std::string& src, void* param)
{
    *static_cast<T*>(param) = str2num<T>(src);
}



//this macro allows to use copy-paste easy to cpp file, it will be redefined later in main cpp file
//if anything added here, it MUST be copy-pasted to Litter_detect.cpp

#define DECLARE_PARAM(TYPE, NAME) extern TYPE NAME
//important parameters
DECLARE_PARAM(float, staticness_th) ; // Staticness score threshold (degree of the object being static)
DECLARE_PARAM(double, objectness_th); //  Objectness score threshold (probability that the rectangle contain an object)
DECLARE_PARAM(uint8_t, aotime); //aotime*framemod2= number of frames the objects must be  static// if set too low, maybe cause false detections !!
DECLARE_PARAM(uint8_t, aotime2); // Half or more of aotime
DECLARE_PARAM(double, alpha); // background scene learning rate
DECLARE_PARAM(double, fore_th); // Threshold moving edges segmentation


//Less important parameters
DECLARE_PARAM(bool, low_light); // if night scene
DECLARE_PARAM(fullbits_int_t, frameinit); // Frames needed for learning the background
DECLARE_PARAM(fullbits_int_t, framemod);
DECLARE_PARAM(fullbits_int_t, framemod2);
DECLARE_PARAM(fullbits_int_t, minsize); // Static object minimum size
DECLARE_PARAM(float, resize_scale); // Image resize scale

#undef DECLARE_PARAM



// trim from start (in place)
inline void ltrim(std::string &s)
{
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
}

// trim from end (in place)
inline void rtrim(std::string &s)
{
    s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}

// trim from both ends (in place)
inline void trim(std::string &s)
{
    ltrim(s);
    rtrim(s);
}

// trim from start (copying)
inline std::string ltrim_copy(std::string s)
{
    ltrim(s);
    return s;
}

// trim from end (copying)
inline std::string rtrim_copy(std::string s)
{
    rtrim(s);
    return s;
}

// trim from both ends (copying)
inline std::string trim_copy(std::string s)
{
    trim(s);
    return s;
}
