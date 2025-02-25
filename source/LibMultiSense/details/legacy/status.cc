/**
 * @file status.cc
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
 *   2025-01-29, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include "details/legacy/status.hh"

namespace multisense {
namespace legacy {

bool system_ok(const crl::multisense::details::wire::StatusResponse &status)
{
    using namespace crl::multisense::details;

    return (status.status & wire::StatusResponse::STATUS_GENERAL_OK) == wire::StatusResponse::STATUS_GENERAL_OK;
}

template <>
MultiSenseStatus::TemperatureStatus convert(const crl::multisense::details::wire::StatusResponse &status)
{
    return MultiSenseStatus::TemperatureStatus{status.temperature1,
                                               status.temperature2,
                                               status.temperature3,
                                               status.temperature0};
}

template <>
MultiSenseStatus::PowerStatus convert(const crl::multisense::details::wire::StatusResponse &status)
{
    return MultiSenseStatus::PowerStatus{status.inputVolts,
                                         status.inputCurrent,
                                         status.fpgaPower};
}

template <>
MultiSenseStatus::CameraStatus convert(const crl::multisense::details::wire::StatusResponse &status)
{
    using namespace crl::multisense::details;

    return MultiSenseStatus::CameraStatus{
        (status.status & wire::StatusResponse::STATUS_CAMERAS_OK) == wire::StatusResponse::STATUS_CAMERAS_OK,
        (status.status & wire::StatusResponse::STATUS_PIPELINE_OK) == wire::StatusResponse::STATUS_PIPELINE_OK};
}

MultiSenseStatus::PtpStatus convert(const crl::multisense::details::wire::PtpStatusResponse &status)
{
    std::array<uint8_t, 8> grandmaster_id = {0, 0, 0, 0, 0, 0, 0, 0};
    memcpy(grandmaster_id.data(), status.gm_id, sizeof(uint8_t) * grandmaster_id.size());
    return MultiSenseStatus::PtpStatus{status.gm_present != 0,
                                       std::move(grandmaster_id),
                                       std::chrono::nanoseconds{status.gm_offset},
                                       std::chrono::nanoseconds{status.path_delay},
                                       status.steps_removed};
}

}
}
