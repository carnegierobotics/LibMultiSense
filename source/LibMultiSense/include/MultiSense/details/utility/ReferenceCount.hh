/**
 * @file LibMultiSense/details/utility/ReferenceCount.hh
 *
 * Declares a ReferenceCount class for tracking resources.  This is
 * derived from Eric Kratzer's ReferenceCount class, which was
 * developed under project RD1034.
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

#ifndef CRL_MULTISENSE_REFERENCECOUNT_HH
#define CRL_MULTISENSE_REFERENCECOUNT_HH

#include <atomic>
#include <stdint.h>

namespace crl {
namespace multisense {
namespace details {
namespace utility {

class ReferenceCount
{
public:
    ReferenceCount()
        : m_countP(new std::atomic<int32_t>(1)) {}

    ReferenceCount(const ReferenceCount& rhs)
        : m_countP(rhs.m_countP) { share(); }

    ReferenceCount& operator=(const ReferenceCount& rhs)
    {
        if (this != &rhs) {
            release();
            m_countP = rhs.m_countP;
            share();
        }
        return *this;
    }

    ~ReferenceCount() { release(); }

    bool isShared() const
    {
        return m_countP && (m_countP->load(std::memory_order_acquire) > 1);
    }

    void reset()
    {
        release();
        m_countP = new std::atomic<int32_t>(1);
    }

private:
    std::atomic<int32_t>* m_countP = nullptr;

    void share()
    {
        if (m_countP)
            m_countP->fetch_add(1, std::memory_order_acq_rel);
    }

    void release()
    {
        if (!m_countP) return;

        if (m_countP->fetch_sub(1, std::memory_order_acq_rel) == 1)
            delete m_countP;

        m_countP = nullptr;
    }
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_REFERENCECOUNT_HH */
