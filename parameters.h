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
