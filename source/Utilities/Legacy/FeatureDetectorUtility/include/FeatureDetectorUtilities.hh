/**
 * @file LibMultiSense/FeatureDetectorMeta.hh
 *
 * Copyright 2024-2025
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
 *   2024-22-11, patrick.smith@carnegierobotics.com, IRAD, Created file.
 **/

#ifndef __FEATURE_DETECTOR_UTILITIES_H__
#define __FEATURE_DETECTOR_UTILITIES_H__

#include "FeatureDetectorMessage.hh"
#include "FeatureDetectorMetaMessage.hh"
#include "FeatureDetectorConfig.hh"

using namespace crl::multisense;
using namespace crl::multisense::details;

namespace feature_detector
{

static CRL_CONSTEXPR DataSource Source_Feature_Left            = Source_Secondary_App_Data_0;
static CRL_CONSTEXPR DataSource Source_Feature_Right           = Source_Secondary_App_Data_1;
static CRL_CONSTEXPR DataSource Source_Feature_Aux             = Source_Secondary_App_Data_2;
static CRL_CONSTEXPR DataSource Source_Feature_Rectified_Left  = Source_Secondary_App_Data_3;
static CRL_CONSTEXPR DataSource Source_Feature_Rectified_Right = Source_Secondary_App_Data_4;
static CRL_CONSTEXPR DataSource Source_Feature_Rectified_Aux   = Source_Secondary_App_Data_5;

/** The recommended maximum number of features for full resolution camera operation */
static CRL_CONSTEXPR int RECOMMENDED_MAX_FEATURES_FULL_RES    = 5000;
/** The recommended maximum number of features for quarter resolution camera operation */
static CRL_CONSTEXPR int RECOMMENDED_MAX_FEATURES_QUARTER_RES = 1500;

class FeatureDetectorConfig: public crl::multisense::system::SecondaryAppConfig {

    private:

        FeatureDetectorConfigParams mConfigItems;

    public:
        /**
         * Query the maximum number of features applied to the camera feature detector
         *
         * @return Return the current maximum number of features
         */
        uint32_t numberOfFeatures() const { return mConfigItems.numberOfFeatures; };

        /**
         * Query the status of the feature detector feature grouping
         *
         * @return Return the current feature grouping status
         */
        bool grouping() const { return mConfigItems.grouping; };

        /**
         * Query the status of the feature detector motion detection
         *
         * @return Return the current feature detector motion detection status
         */
        bool motion() const { return mConfigItems.motion; };

        /**
         * Set the maximum number of features applied to the camera feature detector.
         * Current recommended settings.
         * Full    Resolution: 5000 Features @5FPS
         * Quarter Resolution: 1500 Features @15FPS
         *
         * @param numberOfFeatures The maximum number of features.
         */

        void setNumberOfFeatures(const uint32_t &numberOfFeatures)    {

            if (numberOfFeatures > RECOMMENDED_MAX_FEATURES_FULL_RES)
            {
                std::cout << "WARNING: The number of features requested is above recommended level!" << '\n';
                std::cout << "If a performance impact is noticed reduce number of features and/or framerate of camera" << '\n';
                std::cout << "The recommended maximum camera settings when using the feature detector is:" << '\n';
                std::cout << "Quarter Res: 15FPS and 1500 Features" << '\n';
                std::cout << "Full    Res:  5FPS and 5000 Features" << '\n';
            }

            mConfigItems.numberOfFeatures = numberOfFeatures;

        };

        /**
         * Set the feature grouping capability the feature detector
         *
         * @param g The feature grouping to apply to this camera
         */
        void setGrouping(const bool &g)    {
            mConfigItems.grouping = g;
        }

        /**
         * Set the feature motion detection capability of the feature detector
         * Functions to enable motion detection on Octave 3
         *
         *
         * @param m The feature detector motion detector.
         */
        void setMotion(const uint32_t &m)    {
            mConfigItems.motion = m;
        }

        /** Default constructor */
        FeatureDetectorConfig()
        {
            mConfigItems.numberOfFeatures = RECOMMENDED_MAX_FEATURES_QUARTER_RES;
            mConfigItems.grouping = true;
            mConfigItems.motion = 1;
        };

        void serialize( void )
        {
            memcpy(data, &mConfigItems, sizeof(mConfigItems));
            dataLength = sizeof(mConfigItems);
        }

};

#pragma pack(push,1)

struct Feature {
  uint16_t x;
  uint16_t y;
  uint8_t angle;
  uint8_t resp;
  uint8_t octave;
  uint8_t descriptor;
};

struct Descriptor {
  uint32_t d[8]; //Descriptor is 32 bytes
};

#pragma pack(pop)

class Header : public HeaderBase {

public:

    DataSource source;
    int64_t  frameId;
    uint32_t timeSeconds;
    uint32_t timeNanoSeconds;
    int64_t  ptpNanoSeconds;
    uint16_t octaveWidth;
    uint16_t octaveHeight;
    uint16_t numOctaves;
    uint16_t scaleFactor;
    uint16_t motionStatus;
    uint16_t averageXMotion;
    uint16_t averageYMotion;
    uint16_t numFeatures;
    uint16_t numDescriptors;
    std::vector<Feature> * features;
    std::vector<Descriptor> * descriptors;

  Header()
  {
    features = new std::vector<Feature>;
    descriptors = new std::vector<Descriptor>;
  }

  ~Header()
  {
    delete features;
    delete descriptors;
  }
};


/**
 * Provides an interface to convert generic secondary app data to usable feature detector data
 *
 * @param h the application defined feature detector header
 * @param data the generic payload from the secondary application
 * @param len the length of the generic payload
 * @param frameId the frameId for metadata lookup
 *
 * @return A crl::multisense::Status indicating if the callback deregistration
 * succeeded or failed
 */
Status secondaryAppDataExtract(feature_detector::Header &header, const secondary_app::Header &orig)
{
  using namespace crl::multisense::details;

  utility::BufferStreamReader stream(reinterpret_cast<const uint8_t*>(orig.secondaryAppDataP), orig.secondaryAppDataLength);
  FeatureDetector featureDetector(stream, FeatureDetector::VERSION);

  utility::BufferStreamReader metaStream(reinterpret_cast<const uint8_t *>(orig.secondaryAppMetadataP), orig.secondaryAppMetadataLength);
  FeatureDetectorMeta _meta(metaStream, FeatureDetectorMeta::VERSION);


  header.source         = featureDetector.source;
  header.frameId        = _meta.frameId;
  header.timeSeconds    = _meta.timeSeconds;
  header.timeNanoSeconds= _meta.timeNanoSeconds;
  header.ptpNanoSeconds = _meta.ptpNanoSeconds;
  header.octaveWidth    = _meta.octaveWidth;
  header.octaveHeight   = _meta.octaveHeight;
  header.numOctaves     = _meta.numOctaves;
  header.scaleFactor    = _meta.scaleFactor;
  header.motionStatus   = _meta.motionStatus;
  header.averageXMotion = _meta.averageXMotion;
  header.averageYMotion = _meta.averageYMotion;
  header.numFeatures    = featureDetector.numFeatures;
  header.numDescriptors = featureDetector.numDescriptors;

  const size_t startDescriptor=featureDetector.numFeatures*sizeof(Feature);

  uint8_t * dataP = reinterpret_cast<uint8_t *>(featureDetector.dataP);
  for (size_t i = 0; i < featureDetector.numFeatures; i++) {
      feature_detector::Feature f = *reinterpret_cast<feature_detector::Feature *>(dataP + (i * sizeof(Feature)));
      header.features->push_back(f);
  }

  for (size_t j = 0;j < featureDetector.numDescriptors; j++) {
      feature_detector::Descriptor d = *reinterpret_cast<feature_detector::Descriptor *>(dataP + (startDescriptor + (j * sizeof(Descriptor))));
      header.descriptors->push_back(d);
  }

  return Status_Ok;
}

} // namespace util


#endif /* end of include guard: __FEATURE_DETECTOR_UTILITIES_H__ */
