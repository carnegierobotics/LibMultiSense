/**
 * @file LibMultiSense/FeatureDetectorMeta.hh
 *
 * Copyright 2013-2024
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
 *   2024-22-11, patrick.smith@carnegierobotics.com, IRAD, Moved file.
 **/


#ifdef WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN 1
#endif
#endif

#include "FeatureDetectorUtilities.hh"

namespace feature_detector
{

Status secondaryAppDataExtract(feature_detector::Header &header, const secondary_app::Header &orig)
{
  using namespace crl::multisense::details;

  utility::BufferStreamReader stream(reinterpret_cast<const uint8_t*>(orig.secondaryAppDataP), orig.secondaryAppDataLength);
  wire::FeatureDetector featureDetector(stream, wire::FeatureDetector::VERSION);

  // utility::BufferStreamReader metaStream(reinterpret_cast<const uint8_t *>(orig.secondaryAppMetadataP), orig.secondaryAppMetadataLength);
  // wire::FeatureDetectorMeta _meta(metaStream, wire::FeatureDetectorMeta::VERSION);

  header.source         = featureDetector.source;
  // header.frameId        = _meta.frameId;
  // header.timeSeconds    = _meta.timeSeconds;
  // header.timeNanoSeconds= _meta.timeNanoSeconds;
  // header.ptpNanoSeconds = _meta.ptpNanoSeconds;
  // header.octaveWidth    = _meta.octaveWidth;
  // header.octaveHeight   = _meta.octaveHeight;
  // header.numOctaves     = _meta.numOctaves;
  // header.scaleFactor    = _meta.scaleFactor;
  // header.motionStatus   = _meta.motionStatus;
  // header.averageXMotion = _meta.averageXMotion;
  // header.averageYMotion = _meta.averageYMotion;
  header.numFeatures    = featureDetector.numFeatures;
  header.numDescriptors = featureDetector.numDescriptors;

  const size_t startDescriptor=featureDetector.numFeatures*sizeof(wire::Feature);

  uint8_t * dataP = reinterpret_cast<uint8_t *>(featureDetector.dataP);
  for (size_t i = 0; i < featureDetector.numFeatures; i++) {
      feature_detector::Feature f = *reinterpret_cast<feature_detector::Feature *>(dataP + (i * sizeof(wire::Feature)));
      header.features->push_back(f);
  }

  for (size_t j = 0;j < featureDetector.numDescriptors; j++) {
      feature_detector::Descriptor d = *reinterpret_cast<feature_detector::Descriptor *>(dataP + (startDescriptor + (j * sizeof(wire::Descriptor))));
      header.descriptors->push_back(d);
  }

  return Status_Ok;
}

}
