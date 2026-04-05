#ifndef TIME_HPP
#define TIME_HPP

#include <chrono>

//================================//
namespace Time
{
    using Clock = std::chrono::high_resolution_clock;
    using TimePoint = Clock::time_point;
    using Milliseconds = std::chrono::duration<float, std::milli>;

    //================================//
    inline float MillisecondsBetween(TimePoint start, TimePoint end)
    {
        return Milliseconds(end - start).count();
    }

    //================================//
    inline float MillisecondsSince(TimePoint start)
    {
        return MillisecondsBetween(start, Clock::now());
    }
}

#endif // TIME_HPP