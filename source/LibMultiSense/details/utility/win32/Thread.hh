/**
 * @file LibMultiSense/details/utility/win32/Thread.hh
 *
 * This header file is adapted from Eric Kratzer's (and Dan
 * Tascione's?) Utility.h file, which was developed under project
 * RD1013.
 *
 * Copyright 2014
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
 *   2014-11-03, jyakubik@carnegierobotics.com, ????, Created file.
 **/

#ifndef CRL_MULTISENSE_THREAD_HH
#define CRL_MULTISENSE_THREAD_HH

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#include <windows.h>

#include <stdint.h>
#include <errno.h>
#include <string.h>

#include <vector>
#include <deque>

#include "details/utility/Portability.hh"

#include "../Exception.hh"

namespace crl {
namespace multisense {
namespace details {
namespace utility {

//
// Forward declarations.

class ScopedLock;

//
// A simple class to wrap pthread creation and joining

class Thread {
public:

    static CRL_CONSTEXPR uint32_t FLAGS_DETACH = (1 << 0);

    Thread(LPTHREAD_START_ROUTINE functionP,
           void    *contextP=NULL,
           uint32_t flags=0,    
           int32_t  scheduler=-1,
           int32_t  priority=0) : m_flags(flags) {

        //
        // -1 means the user wants default scheduling behavior

        if (-1 != scheduler) {

            //
            // Set our scheduling policy

            if (scheduler != 0 /* SCHED_OTHER */)
                CRL_EXCEPTION("This platform only supports SCHED_OTHER");

            if (priority != 0)
                CRL_EXCEPTION("Priority can not be set at this time.");
        }

        //
        // Finally, create the thread

        m_threadHandle = CreateThread (NULL, 0, functionP, contextP, 0, &m_threadId);
        if (m_threadHandle == NULL)
            CRL_EXCEPTION("CreateThread() failed: %d", GetLastError());

        //
        // Automatically detach, if asked to do so

        if (FLAGS_DETACH & m_flags)
        {
            CloseHandle (m_threadHandle);
        }
    };

    ~Thread() {
        if (!(m_flags & FLAGS_DETACH) &&
            0 != WaitForSingleObject(m_threadHandle, INFINITE))
            CRL_DEBUG("WaitForSingleObject() failed: %d\n", GetLastError());
    };          
    
private:

    uint32_t  m_flags;
    DWORD m_threadId;
    HANDLE m_threadHandle;
};

//
// A simple mutex class

class Mutex {
public:
    friend class ScopedLock;
    
    Mutex() {
        InitializeCriticalSection(&m_mutex);
    }

    ~Mutex() {
        DeleteCriticalSection(&m_mutex);
    };

private:
    CRITICAL_SECTION m_mutex;
};
    
//
// A simple scoped lock class

class ScopedLock
{
public:

    ScopedLock(Mutex& mutex) {
        this->lock(&mutex.m_mutex);
    };

    ScopedLock(CRITICAL_SECTION *lockP) {
        this->lock(lockP);
    };

    ScopedLock(CRITICAL_SECTION& lock) {
        this->lock(&lock);
    };
        
    ~ScopedLock() {
        LeaveCriticalSection(m_lockP);
    };

private:

    void lock(CRITICAL_SECTION *lockP) {
        m_lockP = lockP;
        EnterCriticalSection(m_lockP);
    };
        
    CRITICAL_SECTION *m_lockP;
};

// A futex-based semaphore.
//
// This implementation does not work across processes.

class Semaphore {
public:

    //
    // Wait for a post (decrement). If thread contention,
    // we may wake up, but be unable to snatch
    // the bait.. hence the while loop.

    bool wait() {        
        do {
            if (0 == wait_(INFINITE))
                return true;
        } while (1);
    };

    //
    // Wait for a post, retrying until timeout

    bool timedWait(const double& timeout) {

        if (timeout < 0.0)
            CRL_EXCEPTION("invalid timeout: %f", timeout);

        do {
            int32_t ret = wait_((DWORD)(timeout * 1000));

            if (0 == ret)
                return true;
            else if (ETIMEDOUT == ret)
                return false;

        } while (1);
    };

    //
    // Post to the semaphore (increment.) Here we
    // signal the futex to wake up any waiters.
    
    bool post() {

        return ReleaseSemaphore(m_handle, 1, NULL) != FALSE;

    };

    //
    // Decrement the semaphore to zero in one-shot.. may
    // fail with thread contention, returns true if
    // successful
    
    bool clear() {
        while(WaitForSingleObject (m_handle, 0) == WAIT_OBJECT_0)
        {
        }
        return true;
    };

    int32_t waiters  () { return m_waiters; };
    bool    decrement() { return wait();    };
    bool    increment() { return post();    };

    Semaphore(std::size_t max=0) :
        m_waiters(0)
    {
        m_handle = CreateSemaphore (NULL, 0, (max == 0 || max > LONG_MAX) ? LONG_MAX : max, NULL);
        if (m_handle == NULL)
            CRL_EXCEPTION ("CreateSemaphore() failed: %d\n", GetLastError());
    }
    
    ~Semaphore()
    {
        if (m_handle != NULL)
            CloseHandle (m_handle);
    }
    
private:

    //
    // This actually does the synchronized decrement if possible, and goes
    // to sleep on the futex if not.

    inline int32_t wait_(DWORD ts=INFINITE) {
        InterlockedIncrement (&m_waiters);
        const int32_t ret = WaitForSingleObject (m_handle, ts);
        InterlockedDecrement (&m_waiters);

        if (ret == WAIT_OBJECT_0)
            return 0;
        else if (ret == WAIT_TIMEOUT)
            return ETIMEDOUT;
        else
            return EAGAIN;
    };
    
    HANDLE m_handle;
    LONG   m_waiters;
};

//
// A templatized variable signaler

template<class T> class WaitVar {
public:

    void post(const T& data) {
        {
            ScopedLock lock(m_lock);
            m_val = data;
        }
        m_sem.post();
    };

    bool wait(T& data) {
        m_sem.wait();
        {
            ScopedLock lock(m_lock);
            data = m_val;
        }
        return true;
    };

    bool timedWait(T& data,
                   const double& timeout) {

        if (false == m_sem.timedWait(timeout))
            return false;
        {
            ScopedLock lock(m_lock);
            data = m_val;
        }
        return true;
    }
    
    //
    // Use a semaphore with max value of 1. The WaitVar will 
    // either be in a signaled state, or not.

    WaitVar() : m_val(),
                m_lock(),
                m_sem(1) {};

private:

    T                 m_val;
    Mutex     m_lock;
    Semaphore m_sem;
};

//
// A templatized wait queue

template <class T> class WaitQueue {
public:

    void post(const T& data) {
        bool postSem=true;
        {
            ScopedLock lock(m_lock);

            //
            // Limit deque size, if requested

            if (m_maximum > 0 && 
                m_maximum == m_queue.size()) {

                //
                // If at max entries, we will pop_front the oldest,
                // push_back the newest, and leave the semaphore alone

                m_queue.pop_front();
                postSem = false;
            }

            m_queue.push_back(data);
        }
        if (postSem) 
            m_sem.post();
    };

    void kick() {
        m_sem.post();
    };

    bool wait(T& data) {
        m_sem.wait();
        {
            ScopedLock lock(m_lock);

            if (0 == m_queue.size())
                return false;
            else {
                data = m_queue.front();
                m_queue.pop_front();
                return true;
            }
        }
    }

    uint32_t waiters() { 
        return m_sem.waiters();
    };

    uint32_t size() {
        ScopedLock lock(m_lock);
        return m_queue.size();
    }
        
    void clear() {
        ScopedLock lock(m_lock);
        m_queue.clear();
        while(false == m_sem.clear());
    }       

    WaitQueue(std::size_t max=0) : 
        m_maximum(max) {};

private:

    const std::size_t m_maximum;
    std::deque<T>     m_queue;
    Mutex             m_lock;
    Semaphore         m_sem;
};

}}}} // namespaces

#endif /* #ifndef CRL_MULTISENSE_THREAD_HH */
