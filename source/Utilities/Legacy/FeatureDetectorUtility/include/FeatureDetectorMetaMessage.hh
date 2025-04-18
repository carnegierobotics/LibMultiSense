/**
 * @file LibMultiSense/FeatureDetectorMetaMessage.hh
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
 *   2024-01-25, patrick.smith@carnegierobotics.com, IRAD, created file.
 *   2024-22-11, patrick.smith@carnegierobotics.com, IRAD, Moved file.
 **/

#ifndef LibMultiSense_FeatureDetectorMetadataMessage
#define LibMultiSense_FeatureDetectorMetadataMessage

#include <typeinfo>

#include "utility/Portability.hh"
#include <MultiSense/MultiSenseChannel.hh>

using namespace crl::multisense::details;

class WIRE_HEADER_ATTRIBS_ FeatureDetectorMetaHeader {
  public:
      static CRL_CONSTEXPR wire::VersionType VERSION    = 2;
      wire::VersionType      version;
      uint32_t               length;
      uint32_t               source;
      int64_t                frameId;
      uint32_t               timeSeconds;
      uint32_t               timeNanoSeconds;
      int64_t                ptpNanoSeconds;
      uint16_t               octaveWidth;
      uint16_t               octaveHeight;
      uint16_t               numOctaves;
      uint16_t               scaleFactor;
      uint16_t               motionStatus;
      uint16_t               averageXMotion;
      uint16_t               averageYMotion;
      uint16_t               numFeatures;
      uint16_t               numDescriptors;

      //
      // Version 2 additions
      uint16_t               observerStatus;
      uint16_t               observerNum;
      uint16_t               observerIndex;
      int16_t                observerDy;
      int16_t                observerTheta;
      uint16_t               affineCalCount;

    FeatureDetectorMetaHeader() :
        version(VERSION),
        length(0),
        source(0),
        frameId(0),
        timeSeconds(0),
        timeNanoSeconds(0),
        ptpNanoSeconds(0),
        octaveWidth(0),
        octaveHeight(0),
        numOctaves(0),
        scaleFactor(0),
        motionStatus(0),
        averageXMotion(0),
        averageYMotion(0),
        numFeatures(0),
        numDescriptors(0),
        observerStatus(0),
        observerNum(0),
        observerIndex(0),
        observerDy(0),
        observerTheta(0),
        affineCalCount(0)
     {};

};

#ifndef SENSORPOD_FIRMWARE

class FeatureDetectorMeta : public FeatureDetectorMetaHeader {
public:

    //
    // Constructors

    FeatureDetectorMeta(utility::BufferStreamReader &r, wire::VersionType v) {serialize(r,v);};
    FeatureDetectorMeta() {};

    //
    // Serialization routine

    template<class Archive>
        void serialize(Archive&          message,
                       const wire::VersionType _version)
    {
        (void) _version;
        message & version;
        message & length;
        message & source;
        message & frameId;
        message & timeSeconds;
        message & timeNanoSeconds;
        message & ptpNanoSeconds;
        message & octaveWidth;
        message & octaveHeight;
        message & numOctaves;
        message & scaleFactor;
        message & motionStatus;
        message & averageXMotion;
        message & averageYMotion;
        message & numFeatures;
        message & numDescriptors;

        if (version >= 2)
        {
          message & observerStatus;
          message & observerNum;
          message & observerIndex;
          message & observerDy;
          message & observerTheta;
          message & affineCalCount;
        }
    }
};

#endif // !SENSORPOD_FIRMWARE

#endif
