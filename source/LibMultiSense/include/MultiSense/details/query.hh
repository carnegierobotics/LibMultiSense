/**
 * @file LibMultiSense/details/query.hh
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
 *   2013-05-15, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef _LibMultiSense_details_query_hh
#define _LibMultiSense_details_query_hh

#include "MultiSense/details/channel.hh"

namespace crl {
namespace multisense {
namespace details {

//
// Publishes the given message to the sensor (message must
// serialize into a single MTU)

template<class T> void impl::publish(const T& message)
{
    const wire::IdType      id      = T::ID;
    const wire::VersionType version = T::VERSION;

    //
    // An output stream to serialize the data

    utility::BufferStreamWriter stream(m_sensorMtu -
                                       wire::COMBINED_HEADER_LENGTH);

    //
    // Hide the header area

    stream.seek(sizeof(wire::Header));

    //
    // Set the ID and version

    stream & id;
    stream & version;

    //
    // Add the message payload. We cast away const here because
    // we have a single serialize() for both directions, and cannot
    // mark it const.

    const_cast<T*>(&message)->serialize(stream, version);

    //
    // Publish the stream

    publish(stream);
}

//
// Send a message, wait for a particular repsonse, re-trying if
// necessary

template <class T> Status impl::waitAck(const T&      msg,
                                        wire::IdType  ackId,
                                        const double& timeout,
                                        int32_t       attempts)
{
    try {
        ScopedWatch ack(ackId, m_watch);

        while(attempts-- > 0) {

            publish(msg);

            Status status;
            if (false == ack.wait(status, timeout))
                continue;
            else
                return status;
        }

        return Status_TimedOut;

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
}

//
// Send a message (with retry), expecting data message,
// extract data payload for user

template <class T, class U> Status impl::waitData(const T&      command,
                                                  U&            data,
                                                  const double& timeout,
                                                  int32_t       attempts)
{
    try {

        //
        // Set up a watch on the command ID in case it is rejected or
        // unsupported.

        ScopedWatch commandAck(T::ID, m_watch);

        //
        // Send the command with retry, expecting the data message as a response.

        Status dataStatus = waitAck(command, MSG_ID(U::ID), timeout, attempts);

        //
        // Also store the response from the command. Do not block, as any
        // response would be registered by this time.

        Status commandStatus;
        if (false == commandAck.wait(commandStatus, 0.0))
            commandStatus = Status_TimedOut;

        //
        // If we did not receive the data message, return the response from
        // the command code instead, unless there was an exception or the
        // command ack'd OK.

        if (Status_Ok != dataStatus) {
            if (Status_Exception == dataStatus ||   // exception
                Status_Ok        == commandStatus)  // data payload timeout or MTU error
                return dataStatus;
            else
                return commandStatus; // command error
        }

        //
        // We have received the data message, extract it for the user.

        return m_messages.extract(data);

    } catch (const std::exception& e) {
        CRL_DEBUG("exception: %s\n", e.what());
        return Status_Exception;
    }
}

}}} // namespaces

#endif // _LibMultiSense_details_publish_hh
