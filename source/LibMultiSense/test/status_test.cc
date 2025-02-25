/**
 * @file status_test.cc
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
 *   2025-01-30, malvarado@carnegierobotics.com, IRAD, Created file.
 **/

#include <gtest/gtest.h>

#include <details/legacy/status.hh>

using namespace multisense::legacy;

crl::multisense::details::wire::StatusResponse create_valid_status()
{
    using namespace crl::multisense::details;

    wire::StatusResponse status;

    status.temperature0 = 1.0f;
    status.temperature1 = 2.0f;
    status.temperature2 = 3.0f;
    status.temperature3 = 4.0f;

    status.inputVolts = 24.0f;
    status.inputCurrent = 1.0f;
    status.fpgaPower = 2.0f;

    status.status |= (wire::StatusResponse::STATUS_GENERAL_OK |
                      wire::StatusResponse::STATUS_CAMERAS_OK |
                      wire::StatusResponse::STATUS_PIPELINE_OK);

    return status;
}

crl::multisense::details::wire::PtpStatusResponse create_valid_ptp_status()
{
    using namespace crl::multisense::details;

    wire::PtpStatusResponse status;
    for (uint8_t i = 0 ; i < 8 ; ++i)
    {
        status.gm_id[i] = i;
    }

    status.gm_present = 1;
    status.gm_offset = 10;
    status.path_delay = 22;
    status.steps_removed = 2;

    return status;
}

TEST(system_ok, not_ok)
{
    using namespace crl::multisense::details;

    wire::StatusResponse status;

    ASSERT_FALSE(system_ok(status));
}


TEST(system_ok, ok)
{
    using namespace crl::multisense::details;

    ASSERT_TRUE(system_ok(create_valid_status()));
}

TEST(convert, temperature)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto status = create_valid_status();

    const auto temp = convert<MultiSenseStatus::TemperatureStatus>(status);

    ASSERT_FLOAT_EQ(temp.fpga_temperature, status.temperature1);
    ASSERT_FLOAT_EQ(temp.left_imager_temperature, status.temperature2);
    ASSERT_FLOAT_EQ(temp.right_imager_temperature, status.temperature3);
    ASSERT_FLOAT_EQ(temp.power_supply_temperature, status.temperature0);
}

TEST(convert, power)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto status = create_valid_status();

    const auto power = convert<MultiSenseStatus::PowerStatus>(status);

    ASSERT_FLOAT_EQ(power.input_voltage, status.inputVolts);
    ASSERT_FLOAT_EQ(power.input_current, status.inputCurrent);
    ASSERT_FLOAT_EQ(power.fpga_power, status.fpgaPower);
}

TEST(convert, camera_status)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto status = create_valid_status();

    const auto camera = convert<MultiSenseStatus::CameraStatus>(status);

    ASSERT_TRUE(camera.cameras_ok);
    ASSERT_TRUE(camera.processing_pipeline_ok);
}

TEST(convert, ptp)
{
    using namespace crl::multisense::details;
    using namespace multisense;

    const auto status = create_valid_ptp_status();

    const auto ptp = convert(status);

    ASSERT_EQ(ptp.grandmaster_present, status.gm_present == 1);

    for (uint8_t i = 0 ; i < ptp.grandmaster_id.size() ; ++i)
    {
        ASSERT_EQ(status.gm_id[i], ptp.grandmaster_id[i]);
    }

    ASSERT_EQ(ptp.grandmaster_offset.count(), status.gm_offset);
    ASSERT_EQ(ptp.path_delay.count(), status.path_delay);
    ASSERT_EQ(ptp.steps_from_local_to_grandmaster, status.steps_removed);
}
