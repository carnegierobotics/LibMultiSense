/**
 * @file LibMultiSense/details/utility/Exception.cc
 *
 * This header file is adapted from Eric Kratzer's (and Dan
 * Tascione's?) StandardException.cc file, which was developed under
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

#include "Exception.hh"

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

#ifndef NEED_VASPRINTF
#define NEED_VASPRINTF 0
#endif

#if NEED_VASPRINTF
int vasprintf (char** strp, const char* fmt, va_list ap)
{
    int len = _vscprintf (fmt, ap);
    if (len < 0)
    {
        *strp = NULL;
        return len;
    }

    *strp = (char*)malloc ((size_t)len + 1);
    if (*strp == NULL)
    {
        return -1;
    }

    len = _vsnprintf (*strp, (size_t)len + 1, fmt, ap);
    if (len < 0)
    {
        free (*strp);
        *strp = NULL;
    }

    return len;
}
#endif

/**
 * Constructor. Initializes with the reason given.
 *
 * \param failureReason The reason for the exception.
 */
Exception::Exception(const char *failureReason, ...)
{
    char   *stringP;
    va_list ap;
    int returnValue;

    va_start(ap, failureReason);
    returnValue = vasprintf(&stringP, failureReason, ap);
    va_end(ap);
    
    if ((NULL != stringP) && (returnValue != -1)) {
        reason = std::string(stringP);
        free(stringP);
    }
}

Exception::Exception(const std::string& failureReason)
{
    reason = failureReason;
}

/**
 * Destructor. Empty.
 */
Exception::~Exception() throw()
{
    // Empty.
}

/**
 * Returns the reason for the exception.
 */
const char* Exception::what() const throw()
{
    return this->reason.c_str();
}

}}}} // namespaces
