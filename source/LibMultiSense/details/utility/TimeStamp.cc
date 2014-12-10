/**
 * @file LibMultiSense/details/utility/TimeStamp.cc
 *
 * The timestamp class gives some type-safety and helper routines for
 * managing time stamps.  This is derived from Dan Tascione's and Eric
 * Kratzer's TimeStamp class, which was developed under project
 * RD1034.
 *
 * Copyright 2011
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

#include "TimeStamp.hh"

#ifndef WIN32
#include <sys/time.h>
#endif
#include <time.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {


// Initialize static routines.

double TimeStamp::timeSynchronizationOffset = 0.0;

#if defined (WIN32)

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
 * Constructor. Initializes with the specified timestamp value.
 */
TimeStamp::TimeStamp(struct timeval& value)
{
    this->set(value);
}

/*
 * Cosntructor. Initializes with the specified timestamp value.
 */
TimeStamp::TimeStamp(double value)
{
    this->time.tv_sec = (int) value;
    this->time.tv_usec = (int) ((value - this->time.tv_sec) * 1000000);
}

/*
 * Sets this timestamp equal to the timestamp specified.
 */
void TimeStamp::set(struct timeval& value)
{
    this->time.tv_sec = value.tv_sec;
    this->time.tv_usec = value.tv_usec;
}

/*
 * Sets the time, for time synchronization. This will report the local clock when
 * the PPS event occurred, and the remote clock when the PPS event occurred.
 *
 * When you use the routine 'getCurrentTime()', it will then return a timestamp
 * that is time-synchronized with the remote system.
 */
void TimeStamp::setTimeAtPps(TimeStamp& local, TimeStamp& remote)
{
    setTimeAtPps(local.time, remote.time);
}

/*
 * Sets the time, for time synchronization. This will report the local clock when
 * the PPS event occurred, and the remote clock when the PPS event occurred.
 *
 * When you use the routine 'getCurrentTime()', it will then return a timestamp
 * that is time-synchronized with the remote system.
 */
void TimeStamp::setTimeAtPps(struct timeval& local, struct timeval& remote)
{
    //
    // To make things atomic, we are somewhat in trouble. To make things lock-free,
    // we are going to do all of the math as doubles. Convert the above timestamps
    // to doubles.
    //

    double localTimeAtPps = local.tv_sec + (local.tv_usec / 1000000.0);
    double remoteTimeAtPps = remote.tv_sec + (remote.tv_usec / 1000000.0);

    //
    // Store the offset between the two as a static variable.
    //

    timeSynchronizationOffset = remoteTimeAtPps - localTimeAtPps;
}

/*
 * Returns the offset from the remote clock to the local clock. Add this to the
 * time returned by a standard time call to have a synchronized time. Notice that
 * this is already applied to anything using TimeStamps normally.
 */
double TimeStamp::getTimeSynchronizationOffset()
{
    return timeSynchronizationOffset;
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
    timeStamp.time.tv_usec = static_cast<long> ((currentTimeAsLargeInteger.QuadPart - timeStamp.time.tv_sec * 10000000) / 10);

#else
    gettimeofday(&timeStamp.time, 0);
#endif

    //
    // Transform it into a double... Notice that this (quite handily)
    // removes all precision up to the 100-nanosecond mark, making carrying
    // times around as timevals fairly pointless. We do this to apply the
    // time synchronization offset, lockless-ly.
    //
    // It's probably that this is pointless, and we should add a lock
    // in the future.
    //

    double currentTime = (double) timeStamp;

    currentTime += timeSynchronizationOffset;

    timeStamp = currentTime;

    //
    // Return the final timestamp.
    //

    return timeStamp;
}

#endif // SENSORPOD_FIRMWARE

/*
 * Returns the seconds portion of the timestamp.
 */
uint32_t TimeStamp::getSeconds() const
{
    return this->time.tv_sec;
}

/*
 * Returns the microseconds portion of the timestamp.
 */
uint32_t TimeStamp::getMicroSeconds() const
{
    return this->time.tv_usec;
}

/*
 * Returns the stored time as if it was a double. This will be seconds since 1970.
 * The time will be accurate to the (roughly) 100-nanosecond mark.
 */
TimeStamp::operator double() const
{
    return this->time.tv_sec + (this->time.tv_usec / 1000000.0);
}

/*
 * Sets the stored time from the seconds value given.
 */
TimeStamp& TimeStamp::operator=(double timeStamp)
{
    this->time.tv_sec = ((unsigned long) timeStamp);
    this->time.tv_usec = ((unsigned long) ((timeStamp - this->time.tv_sec) * 1000000));

    // This call avoids having negative microseconds if the input
    // argument is less than zero and non-integral.
    this->normalize();
    
    return *this;
}


TimeStamp& TimeStamp::operator+=(TimeStamp const& other)
{
    this->time.tv_sec += other.time.tv_sec;
    this->time.tv_usec += other.time.tv_usec;
    this->normalize();
    return *this;
}
  

TimeStamp& TimeStamp::operator-=(TimeStamp const& other)
{
    this->time.tv_sec -= other.time.tv_sec;
    this->time.tv_usec -= other.time.tv_usec;
    this->normalize();
    return *this;
}

  
void TimeStamp::normalize()
{
    while(this->time.tv_usec < 0) {
        this->time.tv_usec += 1000000;
        this->time.tv_sec -= 1;
    }

    while(this->time.tv_usec >= 1000000) {
        this->time.tv_usec -= 1000000;
        this->time.tv_sec += 1;
    }
}


//
// Arithmetic operators are mostly handled by implicit conversion to
// and from double.
//

// TimeStamp operator +(TimeStamp const& arg0, TimeStamp const& arg1)
// {
//   TimeStamp result = arg0;
//   result += arg1;
//   return result;
// }

  
// TimeStamp operator -(TimeStamp const& arg0, TimeStamp const& arg1)
// {
//   TimeStamp result = arg0;
//   result -= arg1;
//   return result;
// }
  
}}}} // namespaces
