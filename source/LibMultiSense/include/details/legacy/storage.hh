/**
 * @file storage.hh
 *
 * Copyright 2013-2025
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
 *   2025-01-08, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#pragma once

#include <memory>
#include <mutex>
#include <vector>

namespace multisense{
namespace legacy{

struct BufferPoolConfig
{
    size_t num_small_buffers = 0;
    size_t small_buffer_size = 0;
    size_t num_large_buffers = 0;
    size_t large_buffer_size = 0;
};

///
/// @brief Object to handle the management and delivery of buffers to used to store incoming data without
///        needing to continually reallocate internal memory. This class is threadsafe
///
class BufferPool
{
public:

    BufferPool(const BufferPoolConfig &config);

    ~BufferPool() = default;

    ///
    /// Non-copyable
    ///
    BufferPool(const BufferPool&) = delete;
    BufferPool& operator=(const BufferPool&) = delete;

    ///
    /// Movable
    ///
    BufferPool(BufferPool&&) noexcept = default;
    BufferPool& operator=(BufferPool&&) noexcept = default;

    ///
    /// @brief Get a buffer which will contain at least target_size bytes of storage
    ///
    std::shared_ptr<std::vector<uint8_t>> get_buffer(size_t target_size);

    ///
    /// @brief Get the current active configuration
    ///
    BufferPoolConfig get_config() const
    {
        return m_config;
    }

private:

    ///
    /// @brief The configured numbers and sizes of our internal buffers
    ///
    BufferPoolConfig m_config;

    ///
    /// @brief The collection of small buffers
    ///
    std::vector<std::shared_ptr<std::vector<uint8_t>>> m_small_buffers;

    ///
    /// @brief The collection of large buffers
    ///
    std::vector<std::shared_ptr<std::vector<uint8_t>>> m_large_buffers;

};

}
}
