/**
 * @file LibMultiSense/details/utility/Portability.hh
 *
 * Macros and symbols to help portability between different compiler
 * versions.
 *
 * Copyright 2014
 * Carnegie Robotics, LLC
 * Ten 40th Street, Pittsburgh, PA 15201
 * http://www.carnegierobotics.com
 *
 * This software is free: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation,
 * version 3 of the License.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * Significant history (date, user, job code, action):
 *   2014-08-07, dlr@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef CRL_MULTISENSE_DETAILS_PORTABILITY_HH
#define CRL_MULTISENSE_DETAILS_PORTABILITY_HH

#if __cplusplus > 199711L
    // This compiler supports C++11.
    #define CRL_CONSTEXPR constexpr
#else
    // This compiler does not support C++11.
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
