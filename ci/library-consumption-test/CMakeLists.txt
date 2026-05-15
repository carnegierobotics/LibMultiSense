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

cmake_minimum_required(VERSION 3.15)

project(multisense_consumption_test LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(CONSUME_JSON_SERIALIZATION
       "Exercise the JSON serialization helpers propagated via find_package(MultiSense)" OFF)
option(CONSUME_OPENCV
       "Exercise the OpenCV-aware helpers propagated via find_package(MultiSense)" OFF)

find_package(MultiSense REQUIRED)

add_executable(multisense_consumption_test main.cc)
target_link_libraries(multisense_consumption_test PRIVATE MultiSense)

if (CONSUME_JSON_SERIALIZATION)
    target_compile_definitions(multisense_consumption_test PRIVATE CONSUME_JSON_SERIALIZATION)
endif()

if (CONSUME_OPENCV)
    target_compile_definitions(multisense_consumption_test PRIVATE CONSUME_OPENCV)
endif()
