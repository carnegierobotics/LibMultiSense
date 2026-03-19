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

std::pair<
    std::vector<std::vector<uint8_t>>,
    std::vector<size_t>>
allocate_buffers( size_t count, size_t buffer_size)
{
    std::vector<std::vector<uint8_t>> storage{};
    std::vector<size_t> free_list{};

    storage.reserve(count);
    for (size_t i = 0 ; i < count ; ++i)
    {
        bool allocated = false;
        for (size_t attempt = 0; attempt < NUM_ALLOCATION_RETRIES; ++attempt)
        {
            try
            {
                storage.emplace_back(buffer_size, 0);
                allocated = true;
                break;
            }
            catch(const std::exception &e)
            {
                CRL_DEBUG("Failed to allocate buffers: %s. Retrying\n", e.what());
            }
        }

        if (!allocated)
        {
            CRL_EXCEPTION("Failed to allocate buffers");
        }
    }

    free_list.reserve(count);
    for (size_t i = 0 ; i < count ; ++i)
    {
        free_list.emplace_back(i);
    }

    return std::make_pair(std::move(storage), std::move(free_list));
}

}

BufferPool::BufferPool(const BufferPoolConfig &config):
    m_config(config)
{
    std::tie(m_small_buffers, m_small_free_list) = allocate_buffers(config.num_small_buffers, config.small_buffer_size);
    std::tie(m_large_buffers, m_large_free_list) = allocate_buffers(config.num_large_buffers, config.large_buffer_size);
}

std::shared_ptr<std::vector<uint8_t>> BufferPool::get_buffer(size_t target_size)
{
    if (target_size <= m_config.small_buffer_size)
    {
        return acquire_buffer(BufferType::Small, target_size);
    }
    else if (target_size <= m_config.large_buffer_size)
    {
        return acquire_buffer(BufferType::Large, target_size);
    }

    return nullptr;
}

std::shared_ptr<std::vector<uint8_t>> BufferPool::acquire_buffer(BufferType type, size_t target_size)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    auto& free_list = (type == BufferType::Small) ? m_small_free_list : m_large_free_list;
    auto& storage   = (type == BufferType::Small) ? m_small_buffers     : m_large_buffers;

    if (free_list.empty())
    {
        return nullptr;
    }

    const size_t index = free_list.back();
    free_list.pop_back();
    auto* buffer = &storage[index];
    lock.unlock();

    buffer->resize(target_size, 0);

    const auto self = shared_from_this();

    return std::shared_ptr<std::vector<uint8_t>>(buffer,
        [self, this, type, index](std::vector<uint8_t>* released_buffer)
        {
            this->release_buffer(type, index, released_buffer);
        });
}

void BufferPool::release_buffer(BufferType type, size_t index, std::vector<uint8_t>* buffer)
{
    const size_t reserve_size = (type == BufferType::Small) ?
                                m_config.small_buffer_size :
                                m_config.large_buffer_size;

    buffer->clear();
    buffer->resize(0);
    buffer->reserve(reserve_size);

    std::lock_guard<std::mutex> lock(m_mutex);

    auto& free_list = (type == BufferType::Small) ? m_small_free_list : m_large_free_list;
    free_list.emplace_back(index);
}

}
}
