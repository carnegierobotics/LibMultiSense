/**
 * @file LibMultiSense/details/utility/Portability.hh
 *
 * Macros and symbols to help portability between different compiler
 * versions.
 *
 * Copyright 2014-2022
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
 *   2014-08-07, dlr@carnegierobotics.com, IRAD, Created file.
 *   2019-06-07, qtorgerson@carnegierobotics.com, converted license to BSD.
 **/

#ifndef CRL_MULTISENSE_DETAILS_PORTABILITY_HH
#define CRL_MULTISENSE_DETAILS_PORTABILITY_HH

#define CRL_UNUSED(var) (void)(var)

#if defined(CRL_HAVE_CONSTEXPR)
    #define CRL_CONSTEXPR constexpr
#else
    #define CRL_CONSTEXPR const
#endif

#if __cplusplus > 199711L
    // Use C++ 11 standard language features.
    #define CRL_THREAD_LOCAL thread_local
#elif defined (__GNUC__)
    // GNU GCC uses __thread to declare thread local variables.
    #define CRL_THREAD_LOCAL __thread
#elif defined (_MSC_VER)
    // MS Visual C++ uses __declspec(thread) to declare thread local variables.
    #define CRL_THREAD_LOCAL __declspec(thread)
#else
    #error "This compiler is not yet supported. Please contact Carnegie Robotics support at http://support.carnegierobotics.com for assistance."
#endif

#if defined (__GNUC__)
    // GNU GCC uses __PRETTY_FUNCTION__ to get the undecorated name of the current function
    // with its type signature.
    #define CRL_PRETTY_FUNCTION __PRETTY_FUNCTION__
#elif defined (_MSC_VER)
    // MS Visual C++ uses __FUNCSIG__ to get the undecorated name of the current function
    #define CRL_PRETTY_FUNCTION __FUNCSIG__
#elif __STDC_VERSION >= 199901L
    // Use C99 standard macros to get the undecorated name of the current function
    #define CRL_PRETTY_FUNCTION __func__
#else
    #error "This compiler is not yet supported. Please contact Carnegie Robotics support at http://support.carnegierobotics.com for assistance."
#endif

#if !defined(MULTISENSE_API)
#if defined (_MSC_VER)
#if defined (MultiSense_EXPORTS)
#define MULTISENSE_API __declspec(dllexport)
#else
#define MULTISENSE_API __declspec(dllimport)
#endif
#else
#define MULTISENSE_API
#endif
#endif

#ifdef WIN32
#ifndef NOMINMAX
#define NOMINMAX 1
#endif

#define usleep(usec) Sleep((usec)/1000)
#endif

#ifndef WIN32
#define closesocket close
#endif

#endif /* #ifndef CRL_MULTISENSE_DETAILS_PORTABILITY_HH */
