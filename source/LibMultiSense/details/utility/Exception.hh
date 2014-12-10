/**
 * @file LibMultiSense/details/utility/Exception.hh
 *
 * This header file is adapted from Eric Kratzer's (and Dan
 * Tascione's?) StandardException.h file, which was developed under
 * project RD1013.
 *
 * Copyright 2012
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
 *   2012-05-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_EXCEPTION_HH
#define CRL_MULTISENSE_EXCEPTION_HH

#include <stdio.h>
#include <string.h>
#include <exception>
#include <string>

#include "TimeStamp.hh"

#ifdef CRL_DEBUG_SYSLOG
#include <syslog.h>
#define CRL_DEBUG_REDIRECTION syslog(LOG_USER|LOG_INFO,
#else
#define CRL_DEBUG_REDIRECTION fprintf(stderr,
#endif // CRL_DEBUG_SYSLOG

#ifdef WIN32
#define CRL_FILENAME                            \
	(strrchr(__FILE__,'\\')                     \
	 ? strrchr(__FILE__,'\\')+1                 \
	 : __FILE__)
#else
#define CRL_FILENAME                            \
    (strrchr(__FILE__,'/')                      \
     ? strrchr(__FILE__,'/')+1                  \
     : __FILE__)
#endif

#define CRL_EXCEPTION(fmt, ...)                                         \
    do {                                                                \
        throw crl::multisense::details::utility::Exception("%s(%d): %s: " fmt,CRL_FILENAME,__LINE__, \
                                                           CRL_PRETTY_FUNCTION,##__VA_ARGS__); \
    } while(0)

#define CRL_DEBUG(fmt, ...)                                             \
    do {                                                                \
        double now = crl::multisense::details::utility::TimeStamp::getCurrentTime(); \
        CRL_DEBUG_REDIRECTION "[%.3f] %s(%d): %s: " fmt,now,CRL_FILENAME,__LINE__, \
                CRL_PRETTY_FUNCTION,##__VA_ARGS__);                     \
    } while(0)


namespace crl {
namespace multisense {
namespace details {
namespace utility {

class Exception : public std::exception
{
private:

    std::string reason;

public:
    
    Exception(const char *failureReason, ...);
    Exception(const std::string& failureReason);
    ~Exception() throw();
    
    virtual const char* what() const throw();
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_EXCEPTION_HH */
