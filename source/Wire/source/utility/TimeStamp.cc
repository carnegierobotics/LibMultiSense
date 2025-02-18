/**
 * @file TimeStamp.cc
 *
 * The timestamp class gives some type-safety and helper routines for
 * managing time stamps.  This is derived from Dan Tascione's and Eric
 * Kratzer's TimeStamp class, which was developed under project
 * RD1034.
 *
 * Copyright 2011-2025
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

#include "utility/TimeStamp.hh"
#include "utility/Exception.hh"

#ifndef WIN32
#include <sys/time.h>
#endif
#include <time.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {
#if defined (WIN32)
ULARGE_INTEGER TimeStamp::offsetSecondsSince1970;
#endif
/*
 * Constructor. Empty. We rely on the getter methods to do
 * things that are more useful.
 */
TimeStamp::TimeStamp()
{
    this->set(0, 0);
}

/*
 * Constructor. Initializes with the specified
 */
TimeStamp::TimeStamp(int32_t seconds, int32_t microSeconds)
{
    this->set(seconds, microSeconds);
}

/*
 * Constructor. Initializes with the specified
 */
TimeStamp::TimeStamp(int64_t nanoseconds)
{
    const int64_t totalMicroSeconds = nanoseconds / 1000;

    const int64_t seconds = totalMicroSeconds / 1000000;
    const int64_t microSeconds = totalMicroSeconds - (seconds * 1000000);

    this->set(static_cast<int32_t>(seconds), static_cast<int32_t>(microSeconds));
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
    set(value.tv_sec, value.tv_usec);
}

/*
 * Sets this timestamp to a specific time
 */
void TimeStamp::set(int32_t seconds, int32_t microSeconds)
{
    const int32_t rollover_sec = microSeconds / 1000000;

    // Convention is for tv_usec to be between 0 and 999999
    // Handle rollover by moving seconds to the
    if (rollover_sec != 0)
    {
        seconds += rollover_sec;
        microSeconds %= 1000000;
    }

    if (microSeconds < 0)
    {
        // we know that abs(tv_usec) is less than 1,000,000 at this point
        seconds -= 1;
        microSeconds += 1000000;
    }

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

    // convert time to nanoseconds
    timeStamp = TimeStamp(static_cast<int64_t>(currentTimeAsLargeInteger.QuadPart) * 100);

#else

#if defined (USE_MONOTONIC_CLOCK)
    struct timespec ts;
    ts.tv_sec = 0;
    ts.tv_nsec = 0;

    if (0 != clock_gettime(CLOCK_MONOTONIC, &ts))
    {
        CRL_EXCEPTION("Failed to call clock_gettime().");
    }

    TIMESPEC_TO_TIMEVAL(&timeStamp.time, &ts);
#else
    gettimeofday(&timeStamp.time, 0);
#endif

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

/*
 * Returns the total time as nanoseconds, aggregates seconds and microseconds
 */
int64_t TimeStamp::getNanoSeconds() const
{
    return static_cast<int64_t>(this->time.tv_sec) * 1000000000 + static_cast<int64_t>(this->time.tv_usec) * 1000;
}

TimeStamp TimeStamp::operator+(TimeStamp const& other) const
{
    // newly constructed TimeStamp handles usec rollover
    return {static_cast<int32_t>(this->time.tv_sec + other.time.tv_sec),
            static_cast<int32_t>(this->time.tv_usec + other.time.tv_usec)};
}

TimeStamp TimeStamp::operator-(TimeStamp const& other) const
{
    // newly constructed TimeStamp handles usec rollover
    return {static_cast<int32_t>(this->time.tv_sec - other.time.tv_sec),
            static_cast<int32_t>(this->time.tv_usec - other.time.tv_usec)};
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
