#!/usr/bin/env python
#
# @file thermal_cal_utility.py
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
from pathlib import Path

import libmultisense as lms


def print_calibration(calibration):
    state = "STAGED (applies at next pipeline start)" if calibration.staged else "active"
    print(f"imager {calibration.imager_id}: {state}")


def get_calibration(channel, imager, calibration_file):
    calibration = lms.secondary_application.thermal.get_calibration(channel, imager)
    if calibration is None:
        raise RuntimeError(f"failed to read calibration for imager {imager}")

    print_calibration(calibration)
    serialized = lms.secondary_application.thermal.serialize_calibration(
        calibration.calibration
    )
    if calibration_file is None:
        print(serialized, end="")
    else:
        with calibration_file.open("w", encoding="utf-8", newline="") as output:
            output.write(serialized)
        print(f"wrote {calibration_file}")


def set_calibration(channel, action, imager, calibration_file):
    try:
        contents = calibration_file.read_text(encoding="utf-8")
    except OSError as error:
        raise RuntimeError(f"failed to read {calibration_file}: {error}") from error

    calibration = lms.secondary_application.thermal.deserialize_calibration(contents)
    if calibration is None:
        raise RuntimeError(f"invalid calibration in {calibration_file}")

    expected = lms.secondary_application.thermal.serialize_calibration(calibration)
    status = lms.secondary_application.thermal.set_calibration(channel, imager, calibration)
    if status != lms.Status.OK:
        raise RuntimeError(f"failed to upload calibration for imager {imager}: {status}")
    print(f"uploaded calibration to imager {imager}")

    if action == "set":
        return

    readback = lms.secondary_application.thermal.get_calibration(channel, imager)
    if readback is None:
        raise RuntimeError("verify failed: could not read calibration back")

    print_calibration(readback)
    actual = lms.secondary_application.thermal.serialize_calibration(readback.calibration)
    if actual != expected:
        raise RuntimeError("verify failed: readback calibration differs")
    print("Verify OK: readback calibration matches")


def main():
    parser = argparse.ArgumentParser(
        description="Manage thermal secondary-application calibrations"
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
        "-c", "--action", choices=("get", "set", "verify"), required=True,
        help="Calibration action",
    )
    parser.add_argument(
        "-i", "--imager", type=int, choices=range(6), default=0,
        help="Imager 0-5 in order 0a,0b,1a,1b,2a,2b",
    )
    parser.add_argument(
        "-f", "--file", type=Path,
        help="Required input for set/verify; optional output for get",
    )
    args = parser.parse_args()

    if args.action in ("set", "verify") and args.file is None:
        parser.error(f"action '{args.action}' requires --file")

    channel_config = lms.ChannelConfig()
    channel_config.ip_address = args.ip_address
    channel_config.mtu = args.mtu

    with lms.Channel.create(channel_config) as channel:
        status = channel.start_streams([lms.DataSource.THERMAL])
        if status != lms.Status.OK:
            raise RuntimeError(f"failed to start thermal application: {status}")

        try:
            if args.action == "get":
                get_calibration(channel, args.imager, args.file)
            else:
                set_calibration(channel, args.action, args.imager, args.file)
        finally:
            channel.stop_streams([lms.DataSource.THERMAL])


if __name__ == "__main__":
    main()
