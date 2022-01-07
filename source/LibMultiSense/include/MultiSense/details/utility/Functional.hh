/**
 * @file LibMultiSense/details/utility/Functional.hh
 *
 * Declarations of convenience functions and functors.
 *
 * Copyright 2012-2022
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

#ifndef CRL_MULTISENSE_FUNCTIONAL_HH
#define CRL_MULTISENSE_FUNCTIONAL_HH

namespace crl {
namespace multisense {
namespace details {
namespace utility {

template <class Type>
inline bool
approximatelyEqual(Type const& xx, Type const& yy, Type const& epsilon) {
    return (((xx)-(yy) < (epsilon)) && ((xx)-(yy) > -(epsilon)));
}

template <class Type>
inline bool
approxEqual(Type const& xx, Type const& yy, Type const& epsilon) {
    return approximatelyEqual(xx, yy, epsilon);
}

template <class Type>
inline Type
boundValue(Type const& value, Type const& minimum, Type const& maximum) {
    return ((value > maximum) ? maximum : (value < minimum) ? minimum : value);
}

template <class Type>
inline Type
decayedAverage(Type const& previous, Type const& samples, Type const& newest) {
    return (((samples - 1) * previous) + newest) / samples;
}

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_FUNCTIONAL_HH */
