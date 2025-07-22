/**
 * @file channel.cc
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
 *   2024-12-24, malvarado@carnegierobotics.com, IRAD, Created file.
 **/


#include "details/mswebrtc/channel.hh"

namespace multisense {
namespace mswebrtc {

MswebrtcChannel::MswebrtcChannel(const Config &c):impl(create_mswebrtc_impl(c.ip_address.c_str()))
{
}

MswebrtcChannel::~MswebrtcChannel()
{
    destroy_mswebrtc_impl(impl);
}


Status MswebrtcChannel::start_streams(const std::vector<DataSource> &)
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

Status MswebrtcChannel::stop_streams(const std::vector<DataSource> &)
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

void MswebrtcChannel::add_image_frame_callback(std::function<void(const ImageFrame&)>)
{    
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

void MswebrtcChannel::add_imu_frame_callback(std::function<void(const ImuFrame&)>)
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

Status MswebrtcChannel::connect(const Config &)
{
    if (connect_mswebrtc_impl(impl)) {
        return Status::OK;
    }
    else {
        return Status::FAILED;
    }
}

void MswebrtcChannel::disconnect()
{
    disconnect_mswebrtc_impl(impl);
}

std::optional<ImageFrame> MswebrtcChannel::get_next_image_frame()
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

std::optional<ImuFrame> MswebrtcChannel::get_next_imu_frame()
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

MultiSenseConfig MswebrtcChannel::get_config()
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

Status MswebrtcChannel::set_config(const MultiSenseConfig &)
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

StereoCalibration MswebrtcChannel::get_calibration()
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

Status MswebrtcChannel::set_calibration(const StereoCalibration &)
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

MultiSenseInfo MswebrtcChannel::get_info()
{
    char builddate[32] = {};
    MultiSenseInfo out = {};
    get_info_mswebrtc_impl(impl, builddate);
    out.version.firmware_build_date = std::string(builddate);
    return out;
}

Status MswebrtcChannel::set_device_info(const MultiSenseInfo::DeviceInfo &, const std::string &)
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

std::optional<MultiSenseStatus> MswebrtcChannel::get_system_status()
{
    CRL_EXCEPTION("NOT IMPLEMENTED");
}

Status MswebrtcChannel::set_network_config(const MultiSenseInfo::NetworkInfo &settings,
                                         const std::optional<std::string> &bcast_interface)
{
    if (bcast_interface) {
        CRL_DEBUG("bcast_interface not supported in set_network_config for mswebrtc\n");
    }
    if (set_network_mswebrtc_impl(impl, settings.ip_address.c_str(), settings.netmask.c_str(), settings.gateway.c_str())) {
        return multisense::Status::OK;
    }
    else {
        return multisense::Status::FAILED;
    }
}

}
}
