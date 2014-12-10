/**
 * @file LibMultiSense/details/listeners.hh
 *
 * Copyright 2013
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

#ifndef LibMultiSense_impl_listeners
#define LibMultiSense_impl_listeners

#include "MultiSenseTypes.hh"

#include "details/utility/Thread.hh"
#include "details/utility/BufferStream.hh"

namespace crl {
namespace multisense {
namespace details {

//
// For access to a buffer back-end in a dispatch thread

extern CRL_THREAD_LOCAL utility::BufferStream *dispatchBufferReferenceTP;

//
// The dispatch mechanism. Each instance represents a bound
// listener to a datum stream.

template<class THeader, class TCallback>
class Listener {
public:
    
    Listener(TCallback   c,
             DataSource s,
             void      *d,
             uint32_t   m=0)
        : m_callback(c),
          m_sourceMask(s),
          m_userDataP(d),
          m_running(false),
          m_queue(m),
          m_dispatchThreadP(NULL) {
        
        m_running         = true;
        m_dispatchThreadP = new utility::Thread(dispatchThread, this);
    };

    Listener() :
        m_callback(NULL),
        m_sourceMask(0),
        m_userDataP(NULL),
        m_running(false),
        m_queue(),
        m_dispatchThreadP(NULL) {};

    ~Listener() {
        if (m_running) {
            m_running = false;
            m_queue.kick();
            delete m_dispatchThreadP;
        }
    };

    void dispatch(THeader& header) {

        if (header.inMask(m_sourceMask))
            m_queue.post(Dispatch(m_callback,
                                  header,
                                  m_userDataP));
    };

    void dispatch(utility::BufferStream& buffer,
                  THeader&                header) {

        if (header.inMask(m_sourceMask))
            m_queue.post(Dispatch(m_callback,
                                  buffer,
                                  header,
                                  m_userDataP));
    };

    TCallback callback() { return m_callback; };

private:

    //
    // For thread-safe dispatching

    class Dispatch {
    public:

        Dispatch(TCallback  c,
                 THeader&   h,
                 void     *d) :
            m_callback(c),
            m_exposeBuffer(false),
            m_header(h),
            m_userDataP(d) {};

        Dispatch(TCallback               c,
                 utility::BufferStream& b,
                 THeader&                h,
                 void                  *d) :
            m_callback(c),
            m_buffer(b),
            m_exposeBuffer(true),
            m_header(h),
            m_userDataP(d) {};

        Dispatch() :
            m_callback(NULL),
            m_buffer(),
            m_exposeBuffer(false),
            m_header(),
            m_userDataP(NULL) {};

        void operator() (void) {

            if (m_callback) {
                if (m_exposeBuffer)
                    dispatchBufferReferenceTP = &m_buffer;
                m_callback(m_header, m_userDataP);
            }
        };

    private:

        TCallback              m_callback;
        utility::BufferStream m_buffer;
        bool                  m_exposeBuffer;
        THeader                m_header;
        void                 *m_userDataP;
    };

    //
    // The dispatch thread
    //
    // We are penalized with two memory copies of
    // HEADER by std::deque, but the image/lidar data
    // is zero-copy (reference-counted by BufferStream)

#if WIN32
    static DWORD WINAPI dispatchThread(void *argumentP) {
#else
    static void *dispatchThread(void *argumentP) {
#endif
        
        Listener<THeader,TCallback> *selfP = reinterpret_cast< Listener<THeader,TCallback> * >(argumentP);
    
        while(selfP->m_running) {
            try {
                Dispatch d;
                if (false == selfP->m_queue.wait(d))
                    break;
                d();
            } catch (const std::exception& e) {
                CRL_DEBUG("exception invoking image callback: %s\n",
                          e.what());
            } catch ( ... ) {
                CRL_DEBUG("unknown exception invoking image callback\n");
            }
        };

        return NULL;
    }

    //
    // Set by user

    TCallback   m_callback;
    DataSource m_sourceMask;
    void      *m_userDataP;

    //
    // Dispatch mechanism
    
    volatile bool                m_running;
    utility::WaitQueue<Dispatch> m_queue;
    utility::Thread             *m_dispatchThreadP;
};

typedef Listener<image::Header, image::Callback> ImageListener;
typedef Listener<lidar::Header, lidar::Callback> LidarListener;
typedef Listener<pps::Header,   pps::Callback>   PpsListener;
typedef Listener<imu::Header,   imu::Callback>   ImuListener;

}; // namespace details
}; // namespace multisense
}; // namespace crl


#endif //  LibMultiSense_impl_listeners
