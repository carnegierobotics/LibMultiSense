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

#define CONSTEXPR constexpr

#else

// This compiler does not support C++11.

#define CONSTEXPR const

#endif

#endif /* #ifndef CRL_MULTISENSE_DETAILS_PORTABILITY_HH */
