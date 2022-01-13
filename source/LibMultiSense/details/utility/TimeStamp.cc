/**
 * @file LibMultiSense/details/utility/TimeStamp.cc
 *
 * The timestamp class gives some type-safety and helper routines for
 * managing time stamps.  This is derived from Dan Tascione's and Eric
 * Kratzer's TimeStamp class, which was developed under project
 * RD1034.
 *
 * Copyright 2011-2022
 * Carnegie Robotics, LLC
 * 4501 Hatfield Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Robotics, LLC nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Significant history (date, user, job code, action):
 *   2012-08-14, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#include "MultiSense/details/utility/TimeStamp.hh"

#ifndef WIN32
#include <sys/time.h>
#endif
#include <time.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {


#if defined (WIN32)

// Windows probably doesn't provider timeradd or timersub macros, so provide
// a definition here
#ifndef timeradd
#define timeradd(a, b, result)                     \
do                                                 \
{                                                  \
  (result)->tv_sec = (a)->tv_sec + (b)->tv_sec;    \
  (result)->tv_usec = (a)->tv_usec + (b)->tv_usec; \
  if ((result)->tv_usec >= 1000000)                \
  {                                                \
    (result)->tv_sec++;                            \
    (result)->tv_usec -= 1000000;                  \
  }                                                \
} while (0)
#endif

#ifndef timersub
#define timersub(a, b, result)                     \
do                                                 \
{                                                  \
  (result)->tv_sec = (a)->tv_sec - (b)->tv_sec;    \
  (result)->tv_usec = (a)->tv_usec - (b)->tv_usec; \
  if ((result)->tv_usec < 0)                       \
  {                                                \
    (result)->tv_sec--;                            \
    (result)->tv_usec += 1000000;                  \
  }                                                \
} while (0)
#endif

//
// The FILETIME structure in Windows measures time in 100-nanosecond intervals
// since 1601-Jan-01. The timeval structure measures time in second intervals
// since 1970-Jan-01. This function computes the number of seconds (as a double)
// that we need to apply as an offset to ensure that clock times are all tracked
// from the same epoch.
static ULARGE_INTEGER initOffsetSecondsSince1970 ()
{
    SYSTEMTIME epochTimeAsSystemTime = { 1970, 1, 0, 1, 0, 0, 0, 0 };
    FILETIME epochTimeAsFileTime;
    SystemTimeToFileTime (&epochTimeAsSystemTime, &epochTimeAsFileTime);

    ULARGE_INTEGER epochTime;
    epochTime.LowPart = epochTimeAsFileTime.dwLowDateTime;
    epochTime.HighPart = epochTimeAsFileTime.dwHighDateTime;

    return epochTime;
}

ULARGE_INTEGER TimeStamp::offsetSecondsSince1970 = initOffsetSecondsSince1970 ();

#endif

/*
 * Constructor. Empty. We rely on the getter methods to do
 * things that are more useful.
 */
TimeStamp::TimeStamp()
{
    this->time.tv_sec = 0;
    this->time.tv_usec = 0;
}

/*
 * Constructor. Initializes with the specified
 */
TimeStamp::TimeStamp(int32_t seconds, int32_t microSeconds)
{
    this->time.tv_sec = seconds;
    this->time.tv_usec = microSeconds;
}

/*
 * Constructor. Initializes with the specified
 */
TimeStamp::TimeStamp(int64_t nanoseconds)
{
    this->time.tv_sec = (long)(nanoseconds / 1000000000);

    int64_t usec = (nanoseconds - (static_cast<int64_t>(this->time.tv_sec) * 1000000000)) / 1000;
    this->time.tv_usec = static_cast<int32_t>(usec);
}

/*
 * Constructor. Initializes with the specified timestamp value.
 */
TimeStamp::TimeStamp(const struct timeval& value)
{
    this->set(value);
}

/*
 * Sets this timestamp equal to the timestamp specified.
 */
void TimeStamp::set(const struct timeval& value)
{
    this->time.tv_sec = value.tv_sec;
    this->time.tv_usec = value.tv_usec;
}

/*
 * Sets this timestamp to a specific time
 */
void TimeStamp::set(int32_t seconds, int32_t microSeconds)
{
    this->time.tv_sec = seconds;
    this->time.tv_usec = microSeconds;
}

#ifndef SENSORPOD_FIRMWARE

/*
 * This routine will get the current time (as gettimeofday()) and
 * store it off. It is the normal way of initializing time. Notice
 * that there may be large time skips when you call this routine, due
 * to time synchronization jumps.
 *
 * Notice that the timestamp returned by this object *is* atomic. It
 * will not change, even if time synchronization skews things.
 */
TimeStamp TimeStamp::getCurrentTime()
{
    //
    // Create a new timestamp object and fill it in with
    // the current time of day.
    //

    TimeStamp timeStamp;

#if defined (WIN32)

    // gettimeofday does not exist on Windows
    FILETIME currentTimeAsFileTime;
    GetSystemTimeAsFileTime (&currentTimeAsFileTime);

    ULARGE_INTEGER currentTimeAsLargeInteger;
    currentTimeAsLargeInteger.LowPart = currentTimeAsFileTime.dwLowDateTime;
    currentTimeAsLargeInteger.HighPart = currentTimeAsFileTime.dwHighDateTime;
    currentTimeAsLargeInteger.QuadPart -= offsetSecondsSince1970.QuadPart;

    timeStamp.time.tv_sec = static_cast<long> (currentTimeAsLargeInteger.QuadPart / 10000000);
    timeStamp.time.tv_usec = static_cast<long> ((currentTimeAsLargeInteger.QuadPart - static_cast<int64_t>(timeStamp.time.tv_sec) * 10000000) / 10);

#else
    gettimeofday(&timeStamp.time, 0);
#endif

    return timeStamp;
}

#endif // SENSORPOD_FIRMWARE

/*
 * Returns the seconds portion of the timestamp.
 */
int32_t TimeStamp::getSeconds() const
{
    return this->time.tv_sec;
}

/*
 * Returns the microseconds portion of the timestamp.
 */
int32_t TimeStamp::getMicroSeconds() const
{
    return this->time.tv_usec;
}

int64_t TimeStamp::getNanoSeconds() const
{
    return static_cast<int64_t>(this->time.tv_sec) * 1000000000 + static_cast<int64_t>(this->time.tv_usec) * 1000;
}

TimeStamp TimeStamp::operator+(TimeStamp const& other) const
{
    struct timeval tmp_time;
    tmp_time.tv_sec = 0;
    tmp_time.tv_usec = 0;
    timeradd(&this->time, &other.time, &tmp_time);
    return tmp_time;
}

TimeStamp TimeStamp::operator-(TimeStamp const& other) const
{
    struct timeval tmp_time;
    tmp_time.tv_sec = 0;
    tmp_time.tv_usec = 0;
    timersub(&this->time, &other.time, &tmp_time);
    return tmp_time;
}

TimeStamp& TimeStamp::operator+=(TimeStamp const& other)
{
    *this = *this + other;
    return *this;
}

TimeStamp& TimeStamp::operator-=(TimeStamp const& other)
{
    *this = *this - other;
    return *this;
}

}}}} // namespaces
