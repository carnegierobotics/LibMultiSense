/**
 * @file LibMultiSense/details/signal.hh
 *
 * Copyright 2013-2022
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
 *   2013-05-07, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_details_signal_hh
#define LibMultiSense_details_signal_hh

#include "MultiSense/details/utility/Thread.hh"
#include "MultiSense/details/wire/Protocol.hh"
#include "MultiSense/details/wire/AckMessage.hh"

#include <map>

namespace crl {
namespace multisense {
namespace details {

//
// Here we provide a thread-safe, blocking, signaling
// interface for sensor message RX.

class MessageWatch {
public:

    void signal(wire::IdType id,
		Status       status=Status_Ok) {
        utility::ScopedLock lock(m_lock);

        Map::iterator it = m_map.find(id);

        if (m_map.end() != it)
            it->second->post(status);
    };

    void signal(const wire::Ack& ack) {
	signal(ack.command, ack.status);
    };

private:

    friend class ScopedWatch;

    typedef utility::WaitVar<Status>        Signal;
    typedef std::map<wire::IdType, Signal*> Map;

    void insert(wire::IdType type,
		Signal      *signalP) {
        utility::ScopedLock lock(m_lock);

        Map::const_iterator it = m_map.find(type);

        //
        // Hmm.. this will prohibit multiple threads
        // simultaneously commanding the sensor with this
	// message ID.

        if (m_map.end() != it)
            CRL_EXCEPTION("ack signal already set for id=%d", type);

        m_map[type] = signalP;
    };

    void remove(wire::IdType type) {
        utility::ScopedLock lock(m_lock);

        Map::iterator it = m_map.find(type);

        if (m_map.end() == it)
            CRL_EXCEPTION("ack signal not found for id=%d\n", type);

        m_map.erase(it);
    };

    utility::Mutex m_lock;
    Map            m_map;
};

 //
 // Exception-safe [de]registration of signal handlers

class ScopedWatch {
public:

    ScopedWatch(wire::IdType  t,
                MessageWatch& m) : m_id(t), m_map(m) {
	m_map.insert(m_id, &m_signal);
    };

    ~ScopedWatch() {
	m_map.remove(m_id);
    };

    bool wait(Status&       status,
	      const double& timeout) {
	return m_signal.timedWait(status, timeout);
    };

private:

    wire::IdType         m_id;
    MessageWatch&        m_map;
    MessageWatch::Signal m_signal;
};

} // namespace details
} // namespace multisense
} // namespace crl


#endif //  LibMultiSense_details_signal_hh
