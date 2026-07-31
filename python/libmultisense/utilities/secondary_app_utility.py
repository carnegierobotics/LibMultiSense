#!/usr/bin/env python
#
# @file secondary_app_utility.py
#
# Copyright 2026
# Carnegie Robotics, LLC
# 4501 Hatfield Street, Pittsburgh, PA 15201
# http://www.carnegierobotics.com
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Carnegie Robotics, LLC nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse

import numpy as np

import libmultisense as lms


def main():
    parser = argparse.ArgumentParser(
        description="Read thermal images through the generic secondary-application API"
    )
    parser.add_argument(
        "-a", "--ip-address", default="10.66.171.21",
        help="The IPv4 address of the MultiSense",
    )
    parser.add_argument(
        "-m", "--mtu", type=int, default=1500,
        help="The MTU used to communicate with the camera",
    )
    parser.add_argument(
        "-n", "--frame-groups", type=int, default=1,
        help="Number of groups to read; 0 runs until Ctrl+C",
    )
    parser.add_argument(
        "-r", "--rectified", action="store_true",
        help="Request rectified thermal images",
    )
    args = parser.parse_args()

    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        try:
            status = channel.start_streams([lms.DataSource.THERMAL])
            if status != lms.Status.OK:
                raise RuntimeError(f"failed to start thermal stream: {status}")

            if args.rectified:
                control = lms.secondary_application.thermal.Control()
                control.command = (
                    lms.secondary_application.thermal.ControlCommand.SET_RECTIFIED
                )
                control.value = 1
                status = control.send(channel)
                if status != lms.Status.OK:
                    raise RuntimeError(f"failed to configure rectification: {status}")

            config = lms.secondary_application.thermal.Config.get(channel)
            if config is not None:
                image_kind = "rectified" if config.rectified else "raw"
                print(
                    f"thermal config: {config.width}x{config.height} "
                    f"uint{config.bits_per_pixel} {image_kind}, "
                    f"enables={config.imager_enable_mask:#x}"
                )

            received_groups = 0
            while args.frame_groups == 0 or received_groups < args.frame_groups:
                packet = channel.get_next_secondary_application_data()
                if packet is None:
                    continue

                # C++ validates the entire payload and constructs Image views.
                group = lms.secondary_application.thermal.FrameGroup.from_buffer(packet.payload)
                print(
                    f"frame {group.frame_id}: {len(group)} images, "
                    f"timestamp {group.camera_timestamp}"
                    f"{' (PTP)' if group.ptp_locked else ''}"
                )

                for thermal_image in group:
                    # This is a read-only, zero-copy uint8 or uint16 NumPy view.
                    image = thermal_image.image.as_array
                    image_kind = "rectified" if thermal_image.rectified else "raw"
                    print(
                        f"  imager {thermal_image.imager_id}: "
                        f"{image.shape[1]}x{image.shape[0]} {image.dtype} {image_kind}, "
                        f"min={image.min()}, max={image.max()}"
                    )

                received_groups += 1
        except KeyboardInterrupt:
            pass
        finally:
            channel.stop_streams([lms.DataSource.THERMAL])


if __name__ == "__main__":
    main()
