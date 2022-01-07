/**
 * @file LibMultiSense/details/utility/TimeStamp.hh
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

#ifndef CRL_MULTISENSE_TIMESTAMP_HH
#define CRL_MULTISENSE_TIMESTAMP_HH

#if defined(WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif

#include <windows.h>
#include <winsock2.h> // Provides a declaration for struct timeval
#else
#include <sys/time.h>
#endif
#include <stdint.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

//
// This is a simple class that helps manage time for the rest of the system,
// abstracting it away into something that is more usable.
//

class TimeStamp
{
private:

    //
    // The stored time.
    //

    struct timeval time;

    //
    // The time synchronization offset. This offset is recalculated each time
    // we receive a PPS timestamp set. It is applied to all times returned by
    // calling getCurrentTime().
    //

    static double timeSynchronizationOffset;
#if defined (WIN32)
    static ULARGE_INTEGER offsetSecondsSince1970;
#endif

public:

    //
    // Static routines, for the singleton version of the TimeStamp.
    //

    static TimeStamp getCurrentTime();

    static void setTimeAtPps(TimeStamp& local, TimeStamp& remote);
    static void setTimeAtPps(struct timeval& local, struct timeval& remote);
    static double getTimeSynchronizationOffset();

    //
    // Public constructor. Initializes from a timestamp.
    //

    TimeStamp();
    TimeStamp(struct timeval& value);
    TimeStamp(double value);

    //
    // For setting the timestamp.
    //

    void set(struct timeval& value);

    //
    // For getting precise values from the timestamp.
    //

    uint32_t getSeconds() const;
    uint32_t getMicroSeconds() const;

    //
    // Operator overloads, for working with time.
    //

    operator double() const;
    TimeStamp& operator=(double timeStamp);
    TimeStamp& operator+=(TimeStamp const& other);
    TimeStamp& operator-=(TimeStamp const& other);
    
    //
    // Make sure internal state is consistent.  Use this if you set
    // the TimeStamp using a timeval struct that had an unchecked
    // microseconds value, such as a negative number.
    //

    void normalize();
};


//
// Arithmetic operators are mostly handled by implicit conversion to
// and from double.
//

// TimeStamp operator +(TimeStamp const& arg0, TimeStamp const& arg1);
// TimeStamp operator -(TimeStamp const& arg0, TimeStamp const& arg1);
  
}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_TIMESTAMP_HH */
