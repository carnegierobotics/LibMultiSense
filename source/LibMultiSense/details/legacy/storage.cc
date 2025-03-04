/**
 * @file storage.cc
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

#include <algorithm>

#include <utility/Exception.hh>

#include "details/legacy/storage.hh"

namespace multisense{
namespace legacy{

namespace {
    constexpr size_t NUM_ALLOCATION_RETRIES = 5;
}

BufferPool::BufferPool(const BufferPoolConfig &config):
    m_config(config)
{

    for (size_t i = 0 ; i < config.num_small_buffers ; ++i)
    {
        for (size_t t = 0; t < NUM_ALLOCATION_RETRIES ; ++t)
        {
            try
            {
                auto small_buffer = std::make_shared<std::vector<uint8_t>>();
                small_buffer->reserve(config.small_buffer_size);
                m_small_buffers.emplace_back(std::move(small_buffer));
                break;
            }
            catch(const std::exception &e)
            {
                (void) e;
                CRL_DEBUG("Failed to allocate small buffer. Retrying\n");
            }
        }
    }

    for (size_t i = 0 ; i < config.num_large_buffers ; ++i)
    {
        for (size_t t = 0; t < NUM_ALLOCATION_RETRIES ; ++t)
        {
            try
            {
                auto large_buffer = std::make_shared<std::vector<uint8_t>>();
                large_buffer->reserve(config.large_buffer_size);
                m_large_buffers.emplace_back(std::move(large_buffer));
                break;
            }
            catch(const std::exception &e)
            {
                (void) e;
                CRL_DEBUG("Failed to allocate large buffer\n");
            }
        }
    }

    if (m_small_buffers.size() != config.num_small_buffers || m_large_buffers.size() != config.num_large_buffers)
    {
        CRL_EXCEPTION("Failed to allocate buffers");
    }
}

std::shared_ptr<std::vector<uint8_t>> BufferPool::get_buffer(size_t target_size)
{
    if (target_size <= m_config.small_buffer_size)
    {
        const auto &small_buffer = std::find_if(std::begin(m_small_buffers), std::end(m_small_buffers),
                                                [](const auto &small_buffer)
                                                {
                                                    return small_buffer.use_count() == 1;
                                                });

        if (small_buffer != std::end(m_small_buffers))
        {
            (*small_buffer)->resize(target_size, 0);
            return *small_buffer;
        }
    }
    else if (target_size <= m_config.large_buffer_size)
    {
        const auto &large_buffer = std::find_if(std::begin(m_large_buffers), std::end(m_large_buffers),
                                                [](const auto &large_buffer)
                                                {
                                                    return large_buffer.use_count() == 1;
                                                });

        if (large_buffer != std::end(m_large_buffers))
        {
            (*large_buffer)->resize(target_size, 0);
            return *large_buffer;
        }
    }

    return nullptr;
}

}
}
