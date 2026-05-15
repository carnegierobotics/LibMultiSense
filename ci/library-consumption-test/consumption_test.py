#!/usr/bin/env python3

#
# Copyright 2013-2025
# Carnegie Robotics, LLC
# 4501 Hatfield Street, Pittsburgh, PA 15201
# http://www.carnegierobotics.com
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
#     * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#       following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#       following disclaimer in the documentation and/or other materials provided with the distribution.
#     * Neither the name of the Carnegie Robotics, LLC nor the names of its contributors may be used to endorse or
#       promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Significant history (date, user, job code, action):
#   2026-05-15, emusser@carnegierobotics.com, IRAD, Created file.
#

"""LibMultiSense Python library-consumption test.

This is a simple script that imports the installed libmultisense package and exercises two exported methods so that the
native _libmultisense extension is loaded at runtime.

The wheel built by pyproject.toml is always configured with -DBUILD_JSON_SERIALIZATION=ON, so this script also exercises
the JSON serialization bindings (the .json property exposed by the pybind11 layer) to verify that the corresponding C++
helpers are linked into the wheel and reachable from Python.
"""

import libmultisense as lms


def main() -> int:
    status = lms.to_string(lms.Status.OK)
    source = lms.to_string(lms.DataSource.LEFT_RECTIFIED_RAW)

    print(f"libmultisense library consumption test: {status}, {source}")

    #
    # Exercise the JSON serialization helpers.
    #

    # JSON serialization is always enabled in the distributed wheel (see pyproject.toml), so the .json property
    # should be available on serializable types like ChannelConfig.  Exercise it to confirm that the underlying C++
    # nlohmann_json helpers are linked into the wheel.
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    if not hasattr(channel_config, "json"):
        print(
            "libmultisense: ChannelConfig is missing the .json property; JSON serialization not built into wheel"
        )
        return 1

    serialized = channel_config.json
    if serialized.get("ip_address") != channel_config.ip_address:
        print(f"libmultisense: JSON serialization mismatch: {serialized!r}")
        return 1

    print(f"libmultisense JSON serialization consumption: {serialized}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
