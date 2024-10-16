/**
 * @file LibMultiSense/MultiSenseTypes.hh
 *
 * Copyright 2013-2022
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
 *   2013-05-06, ekratzer@carnegierobotics.com, PR1044, Created file.
 **/

#ifndef LibMultiSense_MultiSenseTypes_hh
#define LibMultiSense_MultiSenseTypes_hh

#include <climits>
#include <cstring>
#include <iostream>
#include <stdint.h>
#include <string>
#include <vector>

#if defined (CRL_HAVE_CONSTEXPR)
#define CRL_CONSTEXPR constexpr
#else
#define CRL_CONSTEXPR const
#endif

// Define MULTISENSE_API appropriately. This is needed to correctly link with
// LibMultiSense when it is built as a DLL on Windows.
#if !defined(MULTISENSE_API)
#if defined (_MSC_VER)
#if defined (MultiSense_STATIC)
#define MULTISENSE_API __declspec(dllexport)
#elif defined (MultiSense_EXPORTS)
#define MULTISENSE_API __declspec(dllexport)
#else
#define MULTISENSE_API __declspec(dllimport)
#endif

#else
#define MULTISENSE_API
#endif

#endif

#if defined (_MSC_VER)
/*
 * C4251 warns about exporting classes with members not marked as __declspec(export).
 * It is not easy to work around this without breaking the MultiSense API, but it
 * is safe to ignore this warning as long as the MultiSense DLL is compiled with the
 * same compiler version and STL version.
 */
#pragma warning (push)
#pragma warning (disable: 4251)
#endif

namespace crl {
namespace multisense {

/**
 * Sensor version typedef used to store a given version number
 */
typedef uint32_t VersionType;

/**
 * General status typdef used as a return value for get/set
 * crl::multisense::Channel methods
 */
typedef int32_t  Status;

//
// General status codes

static CRL_CONSTEXPR Status Status_Ok          =  0;
static CRL_CONSTEXPR Status Status_TimedOut    = -1;
static CRL_CONSTEXPR Status Status_Error       = -2;
static CRL_CONSTEXPR Status Status_Failed      = -3;
static CRL_CONSTEXPR Status Status_Unsupported = -4;
static CRL_CONSTEXPR Status Status_Unknown     = -5;
static CRL_CONSTEXPR Status Status_Exception   = -6;

/**
 * Data sources typedef representing the various data sources
 * available from sensors in the MultiSense-S line. Variables of this
 * type are used to query which data sources are supported by a given
 * sensor, and to control the streaming of those datasources from the
 * sensor. DataSource values can be combined using the bitwise OR
 * operator to represent multiple sources.
 */
typedef uint64_t DataSource;

static CRL_CONSTEXPR DataSource Source_Unknown                       = 0;
static CRL_CONSTEXPR DataSource Source_All                           = 0xffffffffffffffff;
static CRL_CONSTEXPR DataSource Source_Raw_Left                      = (1ull<<0);
static CRL_CONSTEXPR DataSource Source_Raw_Right                     = (1ull<<1);
static CRL_CONSTEXPR DataSource Source_Luma_Left                     = (1ull<<2);
static CRL_CONSTEXPR DataSource Source_Luma_Right                    = (1ull<<3);
static CRL_CONSTEXPR DataSource Source_Luma_Rectified_Left           = (1ull<<4);
static CRL_CONSTEXPR DataSource Source_Luma_Rectified_Right          = (1ull<<5);
static CRL_CONSTEXPR DataSource Source_Chroma_Left                   = (1ull<<6);
static CRL_CONSTEXPR DataSource Source_Chroma_Right                  = (1ull<<7);
static CRL_CONSTEXPR DataSource Source_Chroma_Rectified_Aux          = (1ull<<8);
static CRL_CONSTEXPR DataSource Source_Disparity                     = (1ull<<10);
static CRL_CONSTEXPR DataSource Source_Disparity_Left                = (1ull<<10); // same as Source_Disparity
static CRL_CONSTEXPR DataSource Source_Disparity_Right               = (1ull<<11);
static CRL_CONSTEXPR DataSource Source_Disparity_Cost                = (1ull<<12);
static CRL_CONSTEXPR DataSource Source_Jpeg_Left                     = (1ull<<16);
static CRL_CONSTEXPR DataSource Source_Rgb_Left                      = (1ull<<17);
static CRL_CONSTEXPR DataSource Source_Feature_Left                  = (1ull<<18);
static CRL_CONSTEXPR DataSource Source_Feature_Right                 = (1ull<<19);
static CRL_CONSTEXPR DataSource Source_Feature_Aux                   = (1ull<<32);
static CRL_CONSTEXPR DataSource Source_Feature_Rectified_Left        = (1ull<<33);
static CRL_CONSTEXPR DataSource Source_Feature_Rectified_Right       = (1ull<<34);
static CRL_CONSTEXPR DataSource Source_Feature_Rectified_Aux         = (1ull<<35);
static CRL_CONSTEXPR DataSource Source_Ground_Surface_Spline_Data    = (1ull<<20);
static CRL_CONSTEXPR DataSource Source_Ground_Surface_Class_Image    = (1ull<<22);
static CRL_CONSTEXPR DataSource Source_AprilTag_Detections           = (1ull<<23);
static CRL_CONSTEXPR DataSource Source_Lidar_Scan                    = (1ull<<24);
static CRL_CONSTEXPR DataSource Source_Imu                           = (1ull<<25);
static CRL_CONSTEXPR DataSource Source_Pps                           = (1ull<<26);
static CRL_CONSTEXPR DataSource Source_Raw_Aux                       = (1ull<<27);
static CRL_CONSTEXPR DataSource Source_Luma_Aux                      = (1ull<<28);
static CRL_CONSTEXPR DataSource Source_Luma_Rectified_Aux            = (1ull<<29);
static CRL_CONSTEXPR DataSource Source_Chroma_Aux                    = (1ull<<30);
static CRL_CONSTEXPR DataSource Source_Disparity_Aux                 = (1ull<<31);
static CRL_CONSTEXPR DataSource Source_Compressed_Left               = (1ull<<9);
static CRL_CONSTEXPR DataSource Source_Compressed_Right              = (1ull<<13);
static CRL_CONSTEXPR DataSource Source_Compressed_Aux                = (1ull<<14);
static CRL_CONSTEXPR DataSource Source_Compressed_Rectified_Left     = (1ull<<15);
static CRL_CONSTEXPR DataSource Source_Compressed_Rectified_Right    = (1ull<<16);
static CRL_CONSTEXPR DataSource Source_Compressed_Rectified_Aux      = (1ull<<17);

/**
 * Use Roi_Full_Image as the height and width when setting the autoExposureRoi
 * to set the ROI to the full image regardless of the current resolution
 */
static CRL_CONSTEXPR int Roi_Full_Image = 0;
static CRL_CONSTEXPR DataSource Exposure_Default_Source = Source_Luma_Left;
static CRL_CONSTEXPR float Exposure_Default_Target_Intensity = 0.5f;
static CRL_CONSTEXPR float Exposure_Default_Gain = 1.0f;

/**
 * Camera profile typedef representing the various stereo profiles available
 * from newer S27/S30 MultiSense variants. Camera profiles are used to augment
 * and extend the standard set of user accessible camera controls. Multiple
 * camera profiles can be requested at once using the bitwise OR operator.
 */
typedef uint32_t CameraProfile;

/** User has direct control over all settings in the image configuration*/
static CRL_CONSTEXPR CameraProfile User_Control = 0;
/** User would like more detail in the disparity image*/
static CRL_CONSTEXPR CameraProfile Detail_Disparity = (1U<<0);
/** User would like more contrast in images*/
static CRL_CONSTEXPR CameraProfile High_Contrast = (1U<<1);
/** User would like see the auto exposure Regions of Interest drawn on the image*/
static CRL_CONSTEXPR CameraProfile Show_ROIs = (1U<<2);
/** User would like to run spline-based ground surface algorithm on the camera*/
static CRL_CONSTEXPR CameraProfile Ground_Surface = (1U<<3);
/** User would like full resolution images from the aux camera regardless of the requested resolution of the stereo pair.
 *  Warning: This profile will be deprecated in future revisions of the software.*/
static CRL_CONSTEXPR CameraProfile Full_Res_Aux_Cam = (1U<<4);
/** User would like to run apriltag detector on the camera*/
static CRL_CONSTEXPR CameraProfile AprilTag = (1U<<5);

/**
 * Image compression codec typedef indicating the compression scheme which was used on the compressed output streams.
 * Compression is only supported on newer S27/S30 MultiSense variants.
 */
typedef uint32_t ImageCompressionCodec;

/** Image data is compressed with the H.264 Codec*/
static CRL_CONSTEXPR ImageCompressionCodec H264 = 0;

/**
 * Remote head channel defines which channel is used to communicate to a
 * specific component in the Remote Head system.
 * The possible components are:
 * Remote_Head_VPB The Remote Head Vision Processor Board.
 * Remote_Head_0   The Remote Head Camera located in position 0
 * Remote_Head_1   The Remote Head Camera located in position 1
 * Remote_Head_2   The Remote Head Camera located in position 2
 * Remote_Head_3   The Remote Head Camera located in position 3
 */
typedef int16_t RemoteHeadChannel;
/** The Remote Head Vision Processor Board */
static CRL_CONSTEXPR RemoteHeadChannel Remote_Head_VPB         = -1;
/** The Remote Head Camera at position 0*/
static CRL_CONSTEXPR RemoteHeadChannel Remote_Head_0           = 0;
/** The Remote Head Camera at position 1*/
static CRL_CONSTEXPR RemoteHeadChannel Remote_Head_1           = 1;
/** The Remote Head Camera at position 2*/
static CRL_CONSTEXPR RemoteHeadChannel Remote_Head_2           = 2;
/** The Remote Head Camera at position 3*/
static CRL_CONSTEXPR RemoteHeadChannel Remote_Head_3           = 3;
/** Invalid Remote Head position*/
static CRL_CONSTEXPR RemoteHeadChannel Remote_Head_Invalid     = SHRT_MAX;

/** The maximum gain supported*/
static CRL_CONSTEXPR float ImagerGainMax = 1000.0f;

/**
 * Remote head sync group defines a group of remote heads which will have their
 * image captures synchronized.
 * It is currently not possible to synchronize more than 2 pairs of remote heads.
 * Furthermore, it is currently not possible to synchronize one head to multiple heads.
 * Given that there are currently only 4 possible remote head cameras, there are
 * only 2 possible remote head synchronization pairs.
 * It is not possible to synchronize Remote Head Stereo Cameras.
 * Possible components are:
 * Remote_Head_0   The Remote Head Camera located in position 0
 * Remote_Head_1   The Remote Head Camera located in position 1
 * Remote_Head_2   The Remote Head Camera located in position 2
 * Remote_Head_3   The Remote Head Camera located in position 3
 */
struct RemoteHeadSyncGroup {

  /**
   * Default constructor
   */
  RemoteHeadSyncGroup() {};

  /**
   * Constructor to initialize a remote head sync pair
   *
   * @param c The remote head sync pair controller
   *
   * @param r The remote head sync responder channels
   *
   */
  RemoteHeadSyncGroup(const RemoteHeadChannel c,
                      const std::vector<RemoteHeadChannel> &r) :
    controller(c),
    responders(r) {};

  /** The synchronization controller */
  RemoteHeadChannel controller;
  /** The synchronization responders */
  std::vector<RemoteHeadChannel> responders;

};

/*
 * Trigger sources used to determine which input should be used to trigger
 * a frame capture on a device
 */
typedef uint32_t TriggerSource;

/** Default internal trigger source. Corresponds to image::config.setFps() */
static CRL_CONSTEXPR TriggerSource Trigger_Internal    = 0;
/** External OPTO_RX trigger input */
static CRL_CONSTEXPR TriggerSource Trigger_External    = 1;
/** External OPTO_RX trigger input with Inverted Polarity */
static CRL_CONSTEXPR TriggerSource Trigger_External_Inverted    = 2;
/** Syncronize cameras on integer timestamps when using PTP */
static CRL_CONSTEXPR TriggerSource Trigger_PTP    = 3;

/**
 * Base Header class for sensor callbacks
 */
class MULTISENSE_API HeaderBase {
public:
    /**
     * This default implementation of the inMask member function will
     * be overridden by derived classes.
     */
    virtual bool inMask(DataSource mask) { (void) mask; return true; };
    virtual ~HeaderBase() {};
};

namespace image {

/**
 * Class containing image Header information common to all image
 * types. This will be passed to any callback, of type
 * image::Callback, that is subscribed to image data.
 *
 * See crl::multisense::Channel::addIsolatedCallback for more details
 *
 * Example code to extract 8 bit image data from a image header and display it
 * using OpenCV (header.bitsPerPixel = 8)
 * \code{.cpp}
 *  #include <iostream>
 *  #include <stdexcept>
 *  #include <signal.h>
 *  #include <unistd.h>
 *
 *  #include <MultiSenseTypes.hh>
 *  #include <MultiSenseChannel.hh>
 *
 *  //
 *  // Note this example has only been tested under Linux
 *  #include <opencv2/opencv.hpp>
 *
 *  volatile bool doneG = false;
 *
 *  void signalHandler(int sig)
 *  {
 *      std::cerr << "Shutting down on signal: " << strsignal(sig) << std::endl;
 *      doneG = true;
 *  }
 *
 *  class Camera
 *  {
 *      public:
 *          Camera(crl::multisense::Channel* channel);
 *          ~Camera();
 *          void imageCallback(const crl::multisense::image::Header& header);
 *
 *      private:
 *          crl::multisense::Channel* m_channel;
 *  };
 *
 *  namespace {
 *      //
 *      // Shim for the C-style callbacks accepted by
 *      // crl::mulisense::Channel::addIsolatedCallback
 *      void monoCallback(const crl::multisense::image::Header& header, void* userDataP)
 *      { reinterpret_cast<Camera*>(userDataP)->imageCallback(header); }
 *  };
 *
 *
 *  Camera::Camera(crl::multisense::Channel* channel):
 *      m_channel(channel)
 *  {
 *      crl::multisense::Status status;
 *
 *      //
 *      // Attach our monoCallback to our Channel instance. It will get
 *      // called every time there is new Left Luma or Right luma image
 *      // data.
 *      status = m_channel->addIsolatedCallback(monoCallback,
 *                                             crl::multisense::Source_Luma_Left | crl::multisense::Source_Luma_Right,
 *                                             this);
 *
 *      //
 *      // Check to see if the callback was successfully attached
 *      if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to attach isolated callback");
 *      }
 *
 *      //
 *      // Start streaming luma images for the left and right cameras.
 *      m_channel->startStreams(crl::multisense::Source_Luma_Left | crl::multisense::Source_Luma_Right);
 *
 *      //
 *      // Check to see if the streams were sucessfully started
 *      if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to start image streams");
 *      }
 *  }
 *
 *  Camera::~Camera()
 *  {
 *      crl::multisense::Status status;
 *
 *      //
 *      // Remove our isolated callback.
 *      status = m_channel->removeIsolatedCallback(monoCallback);
 *
 *      //
 *      // Check to see if the callback was successfully removed
 *      if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to remove isolated callback");
 *      }
 *
 *      //
 *      // Stop streaming luma images for the left and right cameras
 *      m_channel->stopStreams(crl::multisense::Source_Luma_Left | crl::multisense::Source_Luma_Right);
 *
 *      //
 *      // Check to see if the image streams were successfully stopped
 *      if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to stop streams");
 *      }
 *  }
 *
 *  void Camera::imageCallback(const crl::multisense::image::Header& header)
 *  {
 *      //
 *      // Create a container for the image data
 *      std::vector<uint8_t> imageData;
 *      imageData.resize(header.imageLength);
 *
 *      //
 *      // Copy image data from the header's image data pointer to our
 *      // image container
 *      memcpy(&(imageData[0]), header.imageDataP, header.imageLength);
 *
 *      //
 *      // Create a OpenCV matrix using our image container
 *      cv::Mat_<uint8_t> imageMat(header.height, header.width, &(imageData[0]));
 *
 *      //
 *      // Display the image using OpenCV
 *      cv::namedWindow("Example");
 *      cv::imshow("Example", imageMat);
 *      cv::waitKey(1000./header.framesPerSecond);
 *  }
 *
 *
 *  int main()
 *  {
 *      //
 *      // Setup a signal handler to kill the application
 *      signal(SIGINT, signalHandler);
 *
 *      //
 *      // Instantiate a channel connecting to a sensor at the factory default
 *      // IP address
 *      crl::multisense::Channel* channel;
 *      channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *      channel->setMtu(1500);
 *
 *      try
 *      {
 *          Camera camera(channel);
 *          while(!doneG)
 *          {
 *              usleep(100000);
 *          }
 *      }
 *      catch(std::exception& e)
 *      {
 *          std::cerr << e.what() << std::endl;
 *      }
 *
 *      //
 *      // Destroy the channel instance
 *      crl::multisense::Channel::Destroy(channel);
 *  }
 * \endcode
 */
class MULTISENSE_API Header : public HeaderBase {
public:

    /** DataSource corresponding to imageDataP*/
    DataSource  source;
    /** Bits per pixel in the image */
    uint32_t    bitsPerPixel;
    /** Width of the image */
    uint32_t    width;
    /** Height of the image*/
    uint32_t    height;
    /** Unique ID used to describe an image. FrameIds increase sequentally from the device */
    int64_t     frameId;
    /** The time seconds value corresponding to when  the image was captured*/
    uint32_t    timeSeconds;
    /** The time microseconds value corresponding to when the image was captured*/
    uint32_t    timeMicroSeconds;

    /** The image exposure time in microseconds*/
    uint32_t    exposure;
    /** The imager gain the image was captured with */
    float       gain;
    /** The number of frames per second currently streaming from the device */
    float       framesPerSecond;
    /** The length of the image data stored in imageDataP */
    uint32_t    imageLength;
    /** A pointer to the image data */
    const void *imageDataP;

    /**
     * Default Constructor
     */
    Header()
        : source(Source_Unknown) {};

    /**
     * Member function used to determine if the data contained in the header
     * is contained in a specific image mask
     */
    virtual bool inMask(DataSource mask) { return (mask & source) != 0;};
};

/**
 * Function pointer for receiving callbacks of image data. The data
 * pointed to by the Header argument are guaranteed to remain valid only
 * until the callback returns.
 *
 * To reserve the data pointed to by header, so that it remains valid
 * after the callback has returned, use
 * \code{.cpp}
 *      void crl::multisense::Channel::reserveCallbackBuffer();
 * \endcode
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

class MULTISENSE_API ExposureConfig {
public:
    ExposureConfig():
      m_exposure(10000), m_aeEnabled(true), m_aeMax(5000000),  m_aeDecay(7), m_aeThresh(0.9f),
      m_autoExposureRoiX(0), m_autoExposureRoiY(0),
      m_autoExposureRoiWidth(Roi_Full_Image), m_autoExposureRoiHeight(Roi_Full_Image),
      m_aeTargetIntensity(Exposure_Default_Target_Intensity),
      m_gain(Exposure_Default_Gain) {};

    /**
     * Set the exposure time used to capture images. Note auto exposure
     * must be disabled for this to take effect. Default value: 10000
     *
     * @param e The output exposure time in microseconds [10, 5000000]
     */

    void setExposure          (uint32_t e) { m_exposure  = e; };

    /**
     * Set auto-exposure enable flag. Default value: true
     *
     * @param e A boolean used to enable or disable auto-exposure
     */

    void setAutoExposure      (bool e)     { m_aeEnabled = e; };

    /**
     * Set the desired maximum auto-exposure value. Default value: 5000000
     *
     * @param m The maximum auto-exposure value in microseconds
     */

    void setAutoExposureMax   (uint32_t m) { m_aeMax     = m; };

    /**
     * Set the desired auto-exposure decay rate. Default value: 7
     *
     * @param d The auto-exposure decay rate [0, 20]
     */

    void setAutoExposureDecay (uint32_t d) { m_aeDecay   = d; };

    /**
     * Set the desired auto-exposure target Intensity . Default value: 0.95
     *
     * @param d The auto-exposure target intensity [0.0, 1.0]
     */

    void setAutoExposureTargetIntensity (float d) { m_aeTargetIntensity   = d; };

    /**
     * Set the desired auto-exposure threshold. This is the percentage
     * of the image that should be white. Default value: 0.75
     *
     * @param t The desired auto-exposure threshold [0.0, 1.0]
     */

    void setAutoExposureThresh(float t)    { m_aeThresh  = t; };

    /**
     * Set the desired ROI to use when computing the auto-exposure.
     * x axis is horizontal and y axis is vertical.
     * (0,0) coordinate starts in the upper left corner of the image.
     * If (x + w > image width) or (y + h > image height) the sensor will return an error.
     * Setting to default:(0,0,crl::multisense::Roi_Full_Image,crl::multisense::Roi_Full_Image)
     * will use the entire image for the ROI regardless of the current resolution
     * This feature is only available in sensor firmware version 4.3 and greater
     *
     * @param start_x The X coordinate where the ROI starts
     * @param start_y The Y coordinate where the ROI starts
     * @param width The width of the ROI
     * @param height The height of the ROI
     */

    void setAutoExposureRoi(uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height)
    {
        m_autoExposureRoiX = start_x;
        m_autoExposureRoiY = start_y;
        m_autoExposureRoiWidth = width;
        m_autoExposureRoiHeight = height;
    }

    /**
     * Set the gain applied to the camera
     *
     * @param s The gain to apply to this camera
     */

    void setGain(const float &g)    { m_gain  = g; };

    //
    // Query

    /**
     * Query the current image configuration's exposure setting
     *
     * @return the current image exposure setting in microseconds
     */

    uint32_t exposure          () const { return m_exposure;  };

    /**
     * Query the current image configuration's auto-exposure enable setting
     *
     * @return The current image configuration's auto-exposure enable flag
     */

    bool     autoExposure      () const { return m_aeEnabled; };

    /**
     * Query the current image configuration's maximum auto-exposure value
     *
     * @return The current image configuration's maximum auto-exposure value
     */

    uint32_t autoExposureMax   () const { return m_aeMax;     };

    /**
     * Query the current image configuration's auto-exposure decay rate
     *
     * @return The current configuration's auto-exposure decay rate
     */

    uint32_t autoExposureDecay () const { return m_aeDecay;   };

    /**
     * Query the current image configuration's auto-exposure target Intensity
     *
     * @return The current configuration's auto-exposure decay rate
     */

    float autoExposureTargetIntensity () const { return m_aeTargetIntensity;   };

    /**
     * Query the current image configuration's auto-exposure threshold
     *
     * @return The current image configuration's auto-exposure threshold
     */

    float    autoExposureThresh() const { return m_aeThresh;  };

    /**
     * Query the current image configuration's auto-exposure ROI X value
     *
     * @return The current image configuration's auto-exposure ROI X value
     */
    uint16_t autoExposureRoiX        () const { return m_autoExposureRoiX; };

    /**
     * Query the current image configuration's auto-exposure ROI Y value
     *
     * @return The current image configuration's auto-exposure ROI Y value
     */
    uint16_t autoExposureRoiY        () const { return m_autoExposureRoiY; };

    /**
     * Query the current image configuration's auto-exposure ROI width value
     * Will return crl::multisense::Roi_Full_Image for the default setting,
     * when the ROI covers the entire image regardless of current resolution
     *
     * @return The current image configuration's auto-exposure ROI width value
     */
    uint16_t autoExposureRoiWidth    () const { return m_autoExposureRoiWidth; };

    /**
     * Query the current image configuration's auto-exposure ROI height value
     * Will return crl::multisense::Roi_Full_Image for the default setting,
     * when the ROI covers the entire image regardless of current resolution
     *
     * @return The current image configuration's auto-exposure ROI height value
     */
    uint16_t autoExposureRoiHeight   () const { return m_autoExposureRoiHeight; };

    /**
     * Query the gain applied to the camera
     *
     * @return Return the current image configurations gain
     */
    float gain() const { return m_gain; };

    private:
        uint32_t m_exposure;
        bool     m_aeEnabled;
        uint32_t m_aeMax;
        uint32_t m_aeDecay;
        float    m_aeThresh;

        uint16_t m_autoExposureRoiX;
        uint16_t m_autoExposureRoiY;
        uint16_t m_autoExposureRoiWidth;
        uint16_t m_autoExposureRoiHeight;

        float    m_aeTargetIntensity;
        float    m_gain;
};

/**
 * Class used to store a specific camera configuration. Members in
 * this class are set via get and set methods. The class is used as an input
 * to a channel object to query or set camera parameters.
 *
 * Example code to query an image configuration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a imageConfig instance to store the queried configuration
 *     crl::multisense::image::Config imageConfig;
 *
 *     //
 *     // Query the image configuration from the Channel instance
 *     crl::multisense::Status status = channel->getImageConfig(imageConfig);
 *
 *     //
 *     // Check to see if the configuration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query image configuration");
 *     }
 *
 *     //
 *     // Use the image configuration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set an image configurations:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a imageConfig instance to store the queried configuration
 *     crl::multisense::image::Config imageConfig;
 *
 *     crl::multisense::Status status;
 *
 *     //
 *     // Query the image configuration from the Channel instance
 *     status  = channel->getImageConfig(imageConfig);
 *
 *     //
 *     // Check to see if the configuration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query image configuration");
 *     }
 *
 *     //
 *     // Modify image configuration parameters
 *     // Here we increase the frame rate to 30 FPS
 *     imageConfig.setFps(30.0);
 *
 *     //
 *     // Send the new image configuration to the sensor
 *     status = channel->setImageConfig(imageConfig);
 *
 *     //
 *     // Check to see if the configuration was successfully received by the
 *     // sensor
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set image configuration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 */
class MULTISENSE_API Config {
public:

    //
    // User configurable member functions

    /**
     * Set a desired output resolution. Default value: 1024x544
     *
     * @param w The new resolutions width
     *
     * @param h The new resolutions height
     */

    void setResolution        (uint32_t w,
                               uint32_t h) { m_width=w;m_height=h; };

    /**
     * For stereo sensors, set the desired number of disparities used
     * to search for matching features between the left and right
     * images when computing the output disparity image. Default
     * value: 128
     *
     * @param d The number of disparities
     */

    void setDisparities       (uint32_t d) { m_disparities=d;      };

    /**
     * Set the width of the desired output resolution. Default value:
     * 1024
     *
     * @param w The new resolution's width
     */

    void setWidth             (uint32_t w) { m_width     = w;      };

    /**
     * Set the height of the desired output resolution. Default value:
     * 544
     *
     * @param h The new resolution's height
     */

    void setHeight            (uint32_t h) { m_height    = h;      };

    /**
     * Set the desired output frames per second. Default value: 5
     *
     * @note If an external trigger is selected, this value acts as
     *       an upper bounds to the framerate.
     *
     * @param f The desired frames per second
     */

    void setFps               (float f)    { m_fps       = f;      };

    /**
     * Set the desired output image's gain. Default value: 1
     *
     * @param g The output image gain
     */

    void setGain              (float g)    { m_gain      = g; };

    /**
     * Set the exposure time used to capture images. Note auto exposure
     * must be disabled for this to take effect. Default value: 10000
     *
     * @param e The output exposure time in microseconds [10, 5000000]
     */

    void setExposure          (uint32_t e) { m_exposure.setExposure(e); };

    /**
     * Set auto-exposure enable flag. Default value: true
     *
     * @param e A boolean used to enable or disable auto-exposure
     */

    void setAutoExposure      (bool e)     { m_exposure.setAutoExposure(e); };

    /**
     * Set the desired maximum auto-exposure value. Default value: 5000000
     *
     * @param m The maximum auto-exposure value in microseconds
     */

    void setAutoExposureMax   (uint32_t m) { m_exposure.setAutoExposureMax(m); };

    /**
     * Set the desired auto-exposure decay rate. Default value: 7
     *
     * @param d The auto-exposure decay rate [0, 20]
     */

    void setAutoExposureDecay (uint32_t d) { m_exposure.setAutoExposureDecay(d); };

    /**
     * Set the desired auto-exposure target Intensity. Default value: 0.95
     *
     * @param d The auto-exposure target intensity [0.0, 1.0]
     */

    void setAutoExposureTargetIntensity (float d) { m_exposure.setAutoExposureTargetIntensity(d); };

    /**
     * Set the desired auto-exposure threshold. This is the percentage
     * of the image that should be white. Default value: 0.75
     *
     * @param t The desired auto-exposure threshold [0.0, 1.0]
     */

    void setAutoExposureThresh(float t)    { m_exposure.setAutoExposureThresh(t); };

    /**
     * Set the desired image white-balance. Default value: 1.0 for both
     * r and b
     *
     * @param r The input read white-balance value [0.25, 4.0]
     *
     * @param b The input blue white-balance value [0.25, 4.0]
     */

    void setWhiteBalance            (float r,
                                     float b)    { m_wbRed=r;m_wbBlue=b; };

    /**
     * Set the white-balance enable flag. Default value: true
     *
     * @param e A boolean used to enable or disable white-balance
     */

    void setAutoWhiteBalance        (bool e)     { m_wbEnabled   = e;    };

    /**
     * Set the white-balance decay rate. Default value: 3
     *
     * @param d The white-balance decay rate [0, 20]
     */

    void setAutoWhiteBalanceDecay   (uint32_t d) { m_wbDecay     = d;    };

    /**
     * Set the white-balance threshold. Default value: 0.5
     *
     * @param t The desired white-balance threshold [0.0, 1.0]
     */

    void setAutoWhiteBalanceThresh  (float t)    { m_wbThresh    = t;    };

    /**
     * Set the desired stereo post-filter strength. This is used to
     * filter low confidence stereo data before it is sent to the
     * host. Larger numbers indicate more aggressive filtering.  This
     * feature is only available on sensor firmware versions greater
     * than 3.0.  Default value: 0.5
     *
     * @param s The desired stereo post-filter strength [0.0, 1.0]
     */

    void setStereoPostFilterStrength(float s)    { m_spfStrength = s;    };

    /**
     * Set the HDR enable flag. This feature is only available in sensor
     * firmware version greater than 3.1.  Default value: false. Note
     * enabling HDR will disable image white balance. It may also degrade the
     * stereo performance. It is advised to manually tune exposure and gain
     * settings to achieve desired performance.
     *
     * @param e A boolean used to enable or disable HDR on the camera imagers
     */

    void setHdr                     (bool  e)    { m_hdrEnabled  = e;    };

    /**
     * Set the desired ROI to use when computing the auto-exposure.
     * x axis is horizontal and y axis is vertical.
     * (0,0) coordinate starts in the upper left corner of the image.
     * If (x + w > image width) or (y + h > image height) the sensor will return an error
     * Setting to default:(0,0,crl::multisense::Roi_Full_Image,crl::multisense::Roi_Full_Image)
     * will use the entire image for the ROI regardless of the current resolution
     * This feature is only available in sensor firmware version 4.3 and greater
     *
     * @param start_x The X coordinate where the ROI starts
     * @param start_y The Y coordinate where the ROI starts
     * @param width The width of the ROI
     * @param height The height of the ROI
     */

    void setAutoExposureRoi(uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height)
    {
        m_exposure.setAutoExposureRoi(start_x, start_y, width, height);
    }

    /**
     * Set the operation profile for the camera to use. Profile settings subsume other user settings.
     *
     * @param profile The operation profile to use on the camera
     */
    void setCameraProfile(const CameraProfile &profile)
    {
        m_profile = profile;
    }

    /**
     * Set the gamma correction factor.
     *
     * Gamma is a nonlinear operation used to encode and decode luminance.
     * Typical values for gamma range between 1 and 2.2.
     *
     * @param g the gamma constant applied to the camera sources
     */
    void setGamma(const float g) { m_gamma = g; };

    /**
     * Set the auto exposure gain max.
     *
     * This can be used to add an additional clamp to auto exposure which
     * would limit the maximum analog gain set by the auto exposure algorithm.
     *
     * @param g the max gain constant applied to the camera sources
     */
    void setGainMax(const float g) { m_gainMax = g; };


    //
    // Query

    /**
     * Query the current image configuration's width
     *
     * @return The current image width
     */
    uint32_t width       () const { return m_width;       };

    /**
     * Query the current image configuration's height
     *
     * @return The current image height
     */

    uint32_t height      () const { return m_height;      };

    /**
     * Query the current image configuration's number of disparities
     *
     * @return The current number of disparities used when searching for
     * stereo feature matches
     */

    uint32_t disparities () const { return m_disparities; };

    /**
     * Query the current image configuration's frames-per-second setting
     *
     * @return The current frames per second
     */

    float    fps         () const { return m_fps;         };

    /**
     * Query the current image configuration's gain setting
     *
     * @return The current image gain setting
     */
    float    gain              () const { return m_gain;      };


    /**
     * Query the current image configuration's exposure setting
     *
     * @return the current image exposure setting in microseconds
     */

    uint32_t exposure          () const { return m_exposure.exposure();  };

    /**
     * Query the current image configuration's auto-exposure enable setting
     *
     * @return The current image configuration's auto-exposure enable flag
     */

    bool     autoExposure      () const { return m_exposure.autoExposure(); };

    /**
     * Query the current image configuration's maximum auto-exposure value
     *
     * @return The current image configuration's maximum auto-exposure value
     */

    uint32_t autoExposureMax   () const { return m_exposure.autoExposureMax();     };

    /**
     * Query the current image configuration's auto-exposure decay rate
     *
     * @return The current configuration's auto-exposure decay rate
     */

    uint32_t autoExposureDecay () const { return m_exposure.autoExposureDecay();   };

    /**
     * Query the current image configuration's auto-exposure target intensity
     *
     * @return The current image configuration's auto-exposure target intensity
     */

    float autoExposureTargetIntensity () const { return m_exposure.autoExposureTargetIntensity();   };

    /**
     * Query the current image configuration's auto-exposure threshold
     *
     * @return The current image configuration's auto-exposure threshold
     */

    float    autoExposureThresh() const { return m_exposure.autoExposureThresh();  };

    /**
     * Query the current image configuration's red white-balance setting
     *
     * @return The current image configuration's red white-balance setting
     */

    float    whiteBalanceRed         () const { return m_wbRed;       };

    /**
     * Query the current image configuration's blue white-balance setting
     *
     * @return The current image configuration's blue white-balance setting
     */

    float    whiteBalanceBlue        () const { return m_wbBlue;      };

    /**
     * Query the current image configuration's white-balance enable setting
     *
     * @return The current image configuration's white-balance enable flag
     */

    bool     autoWhiteBalance        () const { return m_wbEnabled;   };

    /**
     * Query the current image configuration's white-balance decay rate
     *
     * @return The current image configuration's white-balance decay rate
     */

    uint32_t autoWhiteBalanceDecay   () const { return m_wbDecay;     };

    /**
     * Query the current image configuration's white-balance threshold
     *
     * @return The current image configuration's white-balance threshold
     */

    float    autoWhiteBalanceThresh  () const { return m_wbThresh;    };

    /**
     * Query the current image configuration's stereo post-filter strength
     *
     * @return The current image configuration's stereo post-filter strength
     */

    float    stereoPostFilterStrength() const { return m_spfStrength; };

    /**
     * Query the current image configuration's HDR enable flag
     *
     * @return The current image configuration's HDR enable flag
     */
    bool     hdrEnabled              () const { return m_hdrEnabled;  };

    /**
     * Query the current image configuration's auto-exposure ROI X value
     *
     * @return The current image configuration's auto-exposure ROI X value
     */
    uint16_t autoExposureRoiX        () const { return m_exposure.autoExposureRoiX(); };

    /**
     * Query the current image configuration's auto-exposure ROI Y value
     *
     * @return The current image configuration's auto-exposure ROI Y value
     */
    uint16_t autoExposureRoiY        () const { return m_exposure.autoExposureRoiY(); };

    /**
     * Query the current image configuration's auto-exposure ROI width value
     * Will return crl::multisense::Roi_Full_Image for the default setting,
     * when the ROI covers the entire image regardless of current resolution
     *
     * @return The current image configuration's auto-exposure ROI width value
     */
    uint16_t autoExposureRoiWidth    () const { return m_exposure.autoExposureRoiWidth(); };

    /**
     * Query the current image configuration's auto-exposure ROI height value
     * Will return crl::multisense::Roi_Full_Image for the default setting,
     * when the ROI covers the entire image regardless of current resolution
     *
     * @return The current image configuration's auto-exposure ROI height value
     */
    uint16_t autoExposureRoiHeight   () const { return m_exposure.autoExposureRoiHeight(); };

    /**
     * Query the current image configurations camera profile
     *
     * @return The current image configurations camera profile
     */
    CameraProfile cameraProfile () const { return m_profile; };

    /**
     * Query the gamma correction factor. Gamma correction factor will be applied
     * to all images.
     *
     * @return Return the gamma factor applied to images
     */
    float gamma() const { return m_gamma; };

    /**
     * Query the gain maximum allowed in the camera.
     *
     * @return A value within the range of 1.0 - Max Gain of the imager
     */
    float gainMax() const { return m_gainMax; };

    //
    // Query camera calibration (read-only)
    //
    // These parameters are adjusted for the current operating resolution of the device.

    /**
     * Query the current camera calibrations rectified projection focal length
     * in the x dimension.
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current camera calibrations focal length in the x dimension
     */

    float fx()    const { return m_fx;    };

    /**
     * Query the current camera calibrations rectified projection focal length
     * in the y dimension.
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current camera calibrations focal length in the y dimension
     */

    float fy()    const { return m_fy;    };

    /**
     * Query the current camera calibrations left rectified image center in the
     * x dimension
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current camera calibrations rectified image center in the
     * x dimension
     */

    float cx()    const { return m_cx;    };

    /**
     * Query the current camera calibrations left rectified image center in the
     * y dimension
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current camera calibrations rectified image center in the
     * y dimension
     */

    float cy()    const { return m_cy;    };

    /**
     * Query the current camera calibrations stereo baseline. This is the
     * component of the vector in the x dimension which points from the right
     * rectified camera frame to the left rectified camera frame
     *
     * @return The x component of the current calibrations stereo baseline
     */

    float tx()    const { return m_tx;    };

    /**
     * Query the current camera calibrations stereo baseline. This is the
     * component of the vector in the y dimension which points from the right
     * rectified camera frame to the left rectified camera frame. For
     * rectified images this value will be 0.
     *
     * @return The y component of the current calibrations stereo baseline
     */

    float ty()    const { return m_ty;    };

    /**
     * Query the current camera calibrations stereo baseline. This is the
     * component of the vector in the z dimension which points from the right
     * rectified frame center to the left rectified camera frame. For
     * rectified images this value will be 0.
     *
     * @return The z component of the current calibrations stereo baseline
     */

    float tz()    const { return m_tz;    };

    /**
     * Query the current camera calibrations roll value. This is the Euler
     * roll angle to rotate the right camera frame into the left camera frame.
     * For rectified images this value will be 0.
     *
     * @return the current camera calibrations roll value
     */

    float roll()  const { return m_roll;  };

    /**
     * Query the current camera calibrations pitch  value. This is the Euler
     * pitch  angle to rotate the right camera frame into the left camera frame.
     * For rectified images this value will be 0.
     *
     * @return the current camera calibrations pitch  value
     */

    float pitch() const { return m_pitch; };

    /**
     * Query the current camera calibrations yaw value. This is the Euler
     * yaw angle to rotate the right camera frame into the left camera frame.
     * For rectified images this value will be 0.
     *
     * @return the current camera calibrations yaw value
     */

    float yaw()   const { return m_yaw;   };

    /**
     * Default constructor for a image configuration. Initializes all image
     * configuration members to their default values
     */
    Config() : m_fps(5.0f), m_gain(1.0f),
               m_exposure(),
               m_wbBlue(1.0f), m_wbRed(1.0f), m_wbEnabled(true), m_wbDecay(3), m_wbThresh(0.5f),
               m_width(1024), m_height(544), m_disparities(128), m_spfStrength(0.5f),
               m_hdrEnabled(false),
               m_profile(User_Control),
               m_gamma(2.0),
               m_fx(0), m_fy(0), m_cx(0), m_cy(0),
               m_tx(0), m_ty(0), m_tz(0), m_roll(0), m_pitch(0), m_yaw(0) {};
private:

    float    m_fps, m_gain;
    ExposureConfig m_exposure;
    float    m_wbBlue;
    float    m_wbRed;
    bool     m_wbEnabled;
    uint32_t m_wbDecay;
    float    m_wbThresh;
    uint32_t m_width, m_height;
    uint32_t m_disparities;
    float    m_spfStrength;
    bool     m_hdrEnabled;
    CameraProfile m_profile;
    float    m_gamma;
    float    m_gainMax;

protected:

    float    m_fx, m_fy, m_cx, m_cy;
    float    m_tx, m_ty, m_tz;
    float    m_roll, m_pitch, m_yaw;
};

class MULTISENSE_API AuxConfig {
public:

    //
    // User configurable member functions


    /**
     * Set the desired output image's gain. Default value: 1
     *
     * @param g The output image gain
     */

    void setGain              (float g)    { m_gain      = g; };

    /**
     * Set the exposure time used to capture images. Note auto exposure
     * must be disabled for this to take effect. Default value: 10000
     *
     * @param e The output exposure time in microseconds [10, 5000000]
     */

    void setExposure          (uint32_t e) { m_exposure.setExposure(e); };

    /**
     * Set auto-exposure enable flag. Default value: true
     *
     * @param e A boolean used to enable or disable auto-exposure
     */

    void setAutoExposure      (bool e)     { m_exposure.setAutoExposure(e); };

    /**
     * Set the desired maximum auto-exposure value. Default value: 5000000
     *
     * @param m The maximum auto-exposure value in microseconds
     */

    void setAutoExposureMax   (uint32_t m) { m_exposure.setAutoExposureMax(m); };

    /**
     * Set the desired auto-exposure decay rate. Default value: 7
     *
     * @param d The auto-exposure decay rate [0, 20]
     */

    void setAutoExposureDecay (uint32_t d) { m_exposure.setAutoExposureDecay(d); };

    /**
     * Set the desired auto-exposure target Intensity. Default value: 0.95
     *
     * @param d The auto-exposure target intensity [0.0, 1.0]
     */

    void setAutoExposureTargetIntensity (float d) { m_exposure.setAutoExposureTargetIntensity(d); };

    /**
     * Set the desired auto-exposure threshold. This is the percentage
     * of the image that should be white. Default value: 0.75
     *
     * @param t The desired auto-exposure threshold [0.0, 1.0]
     */

    void setAutoExposureThresh(float t)    { m_exposure.setAutoExposureThresh(t); };

    /**
     * Set the desired image white-balance. Default value: 1.0 for both
     * r and b
     *
     * @param r The input read white-balance value [0.25, 4.0]
     *
     * @param b The input blue white-balance value [0.25, 4.0]
     */

    void setWhiteBalance            (float r,
                                     float b)    { m_wbRed=r;m_wbBlue=b; };

    /**
     * Set the white-balance enable flag. Default value: true
     *
     * @param e A boolean used to enable or disable white-balance
     */

    void setAutoWhiteBalance        (bool e)     { m_wbEnabled   = e;    };

    /**
     * Set the white-balance decay rate. Default value: 3
     *
     * @param d The white-balance decay rate [0, 20]
     */

    void setAutoWhiteBalanceDecay   (uint32_t d) { m_wbDecay     = d;    };

    /**
     * Set the white-balance threshold. Default value: 0.5
     *
     * @param t The desired white-balance threshold [0.0, 1.0]
     */

    void setAutoWhiteBalanceThresh  (float t)    { m_wbThresh    = t;    };

    /**
     * Set the HDR enable flag. This feature is only available in sensor
     * firmware version greater than 3.1.  Default value: false. Note
     * enabling HDR will disable image white balance. It may also degrade the
     * stereo performance. It is advised to manually tune exposure and gain
     * settings to achieve desired performance.
     *
     * @param e A boolean used to enable or disable HDR on the camera imagers
     */

    void setHdr                     (bool  e)    { m_hdrEnabled  = e;    };

    /**
     * Set the desired ROI to use when computing the auto-exposure.
     * x axis is horizontal and y axis is vertical.
     * (0,0) coordinate starts in the upper left corner of the image.
     * If (x + w > image width) or (y + h > image height) the sensor will return an error
     * Setting to default:(0,0,crl::multisense::Roi_Full_Image,crl::multisense::Roi_Full_Image)
     * will use the entire image for the ROI regardless of the current resolution
     * This feature is only available in sensor firmware version 4.3 and greater
     *
     * @param start_x The X coordinate where the ROI starts
     * @param start_y The Y coordinate where the ROI starts
     * @param width The width of the ROI
     * @param height The height of the ROI
     */

    void setAutoExposureRoi(uint16_t start_x, uint16_t start_y, uint16_t width, uint16_t height)
    {
        m_exposure.setAutoExposureRoi(start_x, start_y, width, height);
    }

    /**
     * Set the operation profile for the camera to use. Profile settings subsume other user settings.
     *
     * @param profile The operation profile to use on the camera
     */
    void setCameraProfile(const CameraProfile &profile)
    {
        m_profile = profile;
    }

    /**
     * Set the gamma correction factor.
     *
     * Gamma is a nonlinear operation used to encode and decode luminance.
     * Typical values for gamma range between 1 and 2.2.
     *
     * @param g the gamma constant applied to the camera sources
     */
    void setGamma(const float g) { m_gamma = g; };

    /**
     * Enable sharpening for the aux luma channel.
     *
     * @param s Set to the value of true to enable or false to disable aux luma sharpening.
     */

    void enableSharpening(const bool &s)    { m_sharpeningEnable  = s; };

    /**
     * Set the sharpening percentage for the aux luma channel.
     *
     * @param s The percentage of sharpening to apply. In the range of 0 - 100
     */

    void setSharpeningPercentage(const float &s)    { m_sharpeningPercentage  = s; };

    /**
     * Set the sharpening limit. The maximum difference in pixels that sharpening is
     * is allowed to change between neighboring pixels. This is useful for clamping
     * the sharpening percentage, while still maintaining a large gain.
     *
     * @param s The percentage of sharpening to apply. In the range of 0 - 100
     */

    void setSharpeningLimit(const uint8_t &s)    { m_sharpeningLimit  = s; };

    /**
     * Set the auto exposure gain max.
     *
     * This can be used to add an additional clamp to auto exposure which
     * would limit the maximum analog gain set by the auto exposure algorithm.
     *
     * @param g the max gain constant applied to the camera sources
     */
    void setGainMax(const float g) { m_gainMax = g; };


    //
    // Query
    //
    //
    // Query camera calibration (read-only)
    //
    // These parameters are adjusted for the current operating resolution of the device.

    /**
     * Query the current aux camera calibration rectified projection focal length
     * in the x dimension.
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current aux camera calibrations focal length in the x dimension
     */

    float fx()    const { return m_fx;    };

    /**
     * Query the current aux camera calibration rectified projection focal length
     * in the y dimension.
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current aux camera calibrations focal length in the y dimension
     */

    float fy()    const { return m_fy;    };

    /**
     * Query the current aux camera calibration aux rectified image center in the
     * x dimension
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current aux camera calibrations rectified image center in the
     * x dimension
     */

    float cx()    const { return m_cx;    };

    /**
     * Query the current aux camera calibration aux rectified image center in the
     * y dimension
     *
     * Note this value is scaled based on the current image resolution
     *
     * @return The current aux camera calibration rectified image center in the
     * y dimension
     */

    float cy()    const { return m_cy;    };

    /**
     * Query the current image configuration's gain setting
     *
     * @return The current image gain setting
     */
    float    gain              () const { return m_gain;      };

    /**
     * Query the current image configuration's exposure setting
     *
     * @return the current image exposure setting in microseconds
     */

    uint32_t exposure          () const { return m_exposure.exposure();  };

    /**
     * Query the current image configuration's auto-exposure enable setting
     *
     * @return The current image configuration's auto-exposure enable flag
     */

    bool     autoExposure      () const { return m_exposure.autoExposure(); };

    /**
     * Query the current image configuration's maximum auto-exposure value
     *
     * @return The current image configuration's maximum auto-exposure value
     */

    uint32_t autoExposureMax   () const { return m_exposure.autoExposureMax();     };

    /**
     * Query the current image configuration's auto-exposure decay rate
     *
     * @return The current configuration's auto-exposure decay rate
     */

    uint32_t autoExposureDecay () const { return m_exposure.autoExposureDecay();   };

    /**
     * Query the current image configuration's auto-exposure target intensity
     *
     * @return The current image configuration's auto-exposure target intensity
     */

    float autoExposureTargetIntensity () const { return m_exposure.autoExposureTargetIntensity();   };

    /**
     * Query the current image configuration's auto-exposure threshold
     *
     * @return The current image configuration's auto-exposure threshold
     */

    float    autoExposureThresh() const { return m_exposure.autoExposureThresh();  };

    /**
     * Query the current image configuration's red white-balance setting
     *
     * @return The current image configuration's red white-balance setting
     */

    float    whiteBalanceRed         () const { return m_wbRed;       };

    /**
     * Query the current image configuration's blue white-balance setting
     *
     * @return The current image configuration's blue white-balance setting
     */

    float    whiteBalanceBlue        () const { return m_wbBlue;      };

    /**
     * Query the current image configuration's white-balance enable setting
     *
     * @return The current image configuration's white-balance enable flag
     */

    bool     autoWhiteBalance        () const { return m_wbEnabled;   };

    /**
     * Query the current image configuration's white-balance decay rate
     *
     * @return The current image configuration's white-balance decay rate
     */

    uint32_t autoWhiteBalanceDecay   () const { return m_wbDecay;     };

    /**
     * Query the current image configuration's white-balance threshold
     *
     * @return The current image configuration's white-balance threshold
     */

    float    autoWhiteBalanceThresh  () const { return m_wbThresh;    };

    /**
     * Query the current image configuration's HDR enable flag
     *
     * @return The current image configuration's HDR enable flag
     */
    bool     hdrEnabled              () const { return m_hdrEnabled;  };

    /**
     * Query the current image configuration's auto-exposure ROI X value
     *
     * @return The current image configuration's auto-exposure ROI X value
     */
    uint16_t autoExposureRoiX        () const { return m_exposure.autoExposureRoiX(); };

    /**
     * Query the current image configuration's auto-exposure ROI Y value
     *
     * @return The current image configuration's auto-exposure ROI Y value
     */
    uint16_t autoExposureRoiY        () const { return m_exposure.autoExposureRoiY(); };

    /**
     * Query the current image configuration's auto-exposure ROI width value
     * Will return crl::multisense::Roi_Full_Image for the default setting,
     * when the ROI covers the entire image regardless of current resolution
     *
     * @return The current image configuration's auto-exposure ROI width value
     */
    uint16_t autoExposureRoiWidth    () const { return m_exposure.autoExposureRoiWidth(); };

    /**
     * Query the current image configuration's auto-exposure ROI height value
     * Will return crl::multisense::Roi_Full_Image for the default setting,
     * when the ROI covers the entire image regardless of current resolution
     *
     * @return The current image configuration's auto-exposure ROI height value
     */
    uint16_t autoExposureRoiHeight   () const { return m_exposure.autoExposureRoiHeight(); };

    /**
     * Query the current image configurations camera profile
     *
     * @return The current image configurations camera profile
     */
    CameraProfile cameraProfile () const { return m_profile; };

    /**
     * Query the gamma correction factor. Gamma correction factor will be applied
     * to all images.
     *
     * @return Return the gamma factor applied to images
     */
    float gamma() const { return m_gamma; };

    /**
     * Query whether sharpening is enabled or not on the aux camera.
     *
     * @return Return true if sharpening is enabled, false if sharpening is disabled.
     */
    bool enableSharpening() const { return m_sharpeningEnable; };

    /**
     * Query the percentage of sharpening applied to the aux luma image.
     *
     * @return A value within the range of 0 - 100
     */
    float sharpeningPercentage() const { return m_sharpeningPercentage; };

    /**
     * Query the limit of sharpening applied to the aux luma image.
     *
     * @return A value within the range of 0 - 255 in
     */
    uint8_t sharpeningLimit() const { return m_sharpeningLimit; };

    /**
     * Query the gain maximum allowed in the camera.
     *
     * @return A value within the range of 1.0 - Max Gain of the imager
     */
    float gainMax() const { return m_gainMax; };

    /**
     * Default constructor for a image configuration. Initializes all image
     * configuration members to their default values
     */
    AuxConfig() : m_gain(1.0f),
               m_exposure(),
               m_wbBlue(1.0f), m_wbRed(1.0f), m_wbEnabled(true), m_wbDecay(3), m_wbThresh(0.5f),
               m_hdrEnabled(false),
               m_profile(User_Control),
               m_gamma(2.0),
               m_sharpeningEnable(false), m_sharpeningPercentage(0.0f), m_sharpeningLimit(0),
               m_gainMax(ImagerGainMax),
               m_fx(0), m_fy(0), m_cx(0), m_cy(0) {};
private:

    float    m_gain;
    ExposureConfig m_exposure;
    float    m_wbBlue;
    float    m_wbRed;
    bool     m_wbEnabled;
    uint32_t m_wbDecay;
    float    m_wbThresh;
    bool     m_hdrEnabled;
    CameraProfile m_profile;
    float    m_gamma;
    bool     m_sharpeningEnable;
    float    m_sharpeningPercentage;
    uint8_t  m_sharpeningLimit;
    float    m_gainMax;
protected:

    float    m_fx, m_fy, m_cx, m_cy;
};

/**
 * Class used For querying/setting camera calibration.
 *
 * Parameters are for the maximum operating resolution of the device:
 *     CMV2000: 2048x1088
 *     CVM4000: 2048x2048
 *
 * Example code to querying the current camera calibration from a sensor:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a image::Calibration instance to store the queried calibration
 *     crl::multisense::image::Calibration imageCalibration;
 *
 *     //
 *     // Query the camera calibration from the Channel instance
 *     crl::multisense::Status status = channel->getImageCalibration(imageCalibration);
 *
 *     //
 *     // Check to see if the calibration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query image calibration");
 *     }
 *
 *     //
 *     // Use the image calibration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set a camera calibration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a image::Calibration instance to store the queried calibration
 *     crl::multisense::image::Calibration imageCalibration;
 *
 *     //
 *     // Initialize left and right calibration matrices.
 *     // In this example we are using arbitrary values. These should be
 *     // parameters generated from stereo calibration routine. All sensors
 *     // ship pre-calibrated so this operation is unnecessary.
 *     imageCalibration.left.M  = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
 *     imageCalibration.right.M = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
 *     imageCalibration.left.D  = { 1, 1, 1, 1, 1, 0, 0, 0 };
 *     imageCalibration.right.D = { 1, 1, 1, 1, 1, 0, 0, 0 };
 *     imageCalibration.left.R  = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
 *     imageCalibration.right.R = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
 *     imageCalibration.left.P  = { {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0} };
 *     imageCalibration.right.P = { {1, 0, 0, 1}, {0, 1, 0, 0}, {0, 0, 1, 0} };
 *
 *     //
 *     // Send the new camera calibration to the sensor
 *     crl::multisense::Status status = channel->setImageCalibration(imageCalibration);
 *
 *     //
 *     // Check to see if the sensor successfully received the new camera calibration
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set image calibration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API Calibration {
public:

    /**
     * Class to store camera calibration matrices.
     */
    class MULTISENSE_API Data {
    public:

        /**Camera un-rectified 3x3 projection matrix */
        float M[3][3];
        /**Camera distortion matrix. Can either by the 5 parameter plumb-bob model
         * or the 8 parameter rational polynomial model */
        float D[8];
        /**Camera 3x3 rectification matrix */
        float R[3][3];
        /**Camera 3x4 rectified projection matrix */
        float P[3][4];
    };

    /**Full resolution camera calibration corresponding to the left camera */
    Data left;
    /**Full resolution camera calibration corresponding to the right camera */
    Data right;
    /**Full resolution camera calibration corresponding to aux color camera */
    Data aux;
};

class MULTISENSE_API TransmitDelay {
public:

    /** Delay between completing a frame and actually sending it over the network (in ms) */
    int delay;

};

class MULTISENSE_API PacketDelay {
public:

    /** Enable interpacket delay, should be used when client network cannot handle inbound messages */
    bool enable;

};

/**
 * Class which stores a image histogram from a camera image. This is used
 * as an input when querying a image histogram.
 *
 * Example code to query a image histogram for a left image corresponding to
 * frameId 100
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a histogram instance to store histogram data
 *     crl::multisense::image::Histogram histogram;
 *
 *     uint64_t frameId = 100;
 *
 *     //
 *     // Query the histogram from the image with the frameId == 100.
 *     // Note histograms can only be queried for images whose frameIds are
 *     // less than 20 frameIds away from the most recent received frameId
 *     crl::multisense::Status status = channel->getImageHistogram(frameId, histogram);
 *
 *     //
 *     // Check to see if the histogram query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query image histogram for frameId %d", frameId);
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API Histogram {
public:

    /**
     * Default constructor
     */
    Histogram() : channels(0),
                  bins(0),
                  data() {};
    /**The number of color channels in the given histogram. For color images
     * this is 4 corresponding to the GRBG Bayer channels */
    uint32_t              channels;
    /**The number of possible pixel values for each color channel */
    uint32_t              bins;
    /**The histogram data concatinated serially in GRBG order. The length of
     * data is equal to channels * bins */
    std::vector<uint32_t> data;
};

class MULTISENSE_API RemoteHeadConfig {
public:

    /**
     * Default constructor with no sync groups
     */
    RemoteHeadConfig() : m_syncGroups()
    {};

    /**
     * Constructor allowing definition of sync groups
     *
     * @param sync_groups A vector of remote head syncronization groups
     */
    RemoteHeadConfig(const std::vector<RemoteHeadSyncGroup> &sync_groups) :
        m_syncGroups(sync_groups)
    {}

    /**
     * Set the groups of remote head cameras to be synchronized
     *
     * @param sync_groups A vector of remote head synchronization groups to set
     */
    void setSyncGroups(const std::vector<RemoteHeadSyncGroup> &sync_groups) { m_syncGroups = sync_groups; }

    /**
     * Query the groups of remote head cameras to be synchronized
     *
     * @return The current remote head synchronization groups
     */
    std::vector<RemoteHeadSyncGroup> syncGroups() const { return m_syncGroups; }

private:

    /**The groups of remote head cameras to be synchronized */
    std::vector<RemoteHeadSyncGroup> m_syncGroups;
};


} // namespace image

namespace lidar {

/** The type of a single laser range measurement  */
typedef uint32_t RangeType;
/** The type of a single laser intensity measurement  */
typedef uint32_t IntensityType;

/**
 * Class which stores Header information for a lidar scan. This will be passed
 * to any isolated callbacks, of type lidar::Callback, subscribed to lidar data.
 *
 * See crl::multisense::Channel::addIsolatedCallback
 */
class MULTISENSE_API Header : public HeaderBase {
public:

    /**
     * Default constructor
     */
    Header()
        : pointCount(0) {};

    /**A unique ID number corresponding a individual laser scan */
    uint32_t scanId;
    /** The seconds value of the time corresponding to the start of laser scan */
    uint32_t timeStartSeconds;
    /** The microseconds value of the time corresponding to the start of laser scan */
    uint32_t timeStartMicroSeconds;
    /** The seconds value of the time corresponding to the end of laser scan */
    uint32_t timeEndSeconds;
    /** The microseconds value of the time corresponding to the end of laser scan */
    uint32_t timeEndMicroSeconds;
    /** The spindle angle in microradians corresponding to the start of a laser scan */
    int32_t  spindleAngleStart;
    /** The spindle angle in microradians corresponding to the end of a laser scan */
    int32_t  spindleAngleEnd;
    /** The total angular range of a individual laser scan in microradians */
    int32_t  scanArc;
    /** The maximum range of the laser scanner in millimeters */
    uint32_t maxRange;
    /** The number of points in the laser scan */
    uint32_t pointCount;

    /** Laser range data in millimeters. The length of rangesP is equal
     * to pointCount */
    const RangeType     *rangesP;
    /** Laser intensity values corresponding to each laser range point. The
     * length of intensitiesP is equal to pointClount */
    const IntensityType *intensitiesP;  // device units
};

/**
 * Function pointer for receiving callbacks of lidar data. Pointers
 * to data are no longer valid after the callback returns.
 *
 * To reserve data pointers after the callback returns use
 * \code{.cpp}
 *     void crl::multisense::Channel::reserveCallbackBuffer();
 * \endcode
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

/**
 * Class used to store a laser calibration. Calibrations can be queried or set
 * for a specific sensor
 *
 * Example code to query the current laser calibration from a sensor:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a lidarCalibration instance to store the queried laser calibration
 *     crl::multisense::lidar::Calibration lidarCalibration;
 *
 *     //
 *     // Query the laser calibration from the current Channel instance
 *     crl::multisense::Status status = channel->getLidarCalibration(lidarCalibration);
 *
 *     //
 *     // Check to see if the laser calibration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query laser calibration");
 *     }
 *
 *     //
 *     // Use the laser calibration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set a laser calibration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a lidarCalibration instance to store the new laser calibration
 *     crl::multisense::lidar::Calibration lidarCalibration;
 *
 *     //
 *     // Populate the laser calibration. Here we populate the calibration
 *     // with identity matrices. All MultiSense SL sensors ship  pre-calibrated
 *     // making this unnecessary.
 *     lidarCalibration.laserToSpindle = { {1, 0, 0, 0},
 *                                         {0, 1, 0, 0},
 *                                         {0, 0, 1, 0},
 *                                         {0, 0, 0, 1} };
 *
 *     lidarCalibration.cameraToSpindleFixed = { {1, 0, 0, 0},
 *                                             {0, 1, 0, 0},
 *                                             {0, 0, 1, 0},
 *                                             {0, 0, 0, 1} };
 *
 *     //
 *     // Send the new laser calibration to the sensor
 *     crl::multisense::Status status = channel->setLidarCalibration(lidarCalibration);
 *
 *     //
 *     // Check to see if the new laser calibration was successfully received by
 *     // the sensor
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set lidar calibration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API Calibration {
public:

    /** A 4x4 homogeneous transform matrix corresponding to the transform from
     * the laser origin coordinate frame to the rotating spindle frame  */
    float laserToSpindle[4][4];
    /** A 4x4 homogeneous transform matrix corresponding to the transform from
     * the static spindle frame to the left camera optical frame */
    float cameraToSpindleFixed[4][4];
};

} // namespace lidar

namespace lighting {

/** The maximum number of lights for a given sensor */
static CRL_CONSTEXPR uint32_t MAX_LIGHTS     = 8;
/** The maximum duty cycle for adjusting light intensity */
static CRL_CONSTEXPR float    MAX_DUTY_CYCLE = 100.0;

/**
 * Class used to store a specific lighting configuration. Member of this class
 * are set and queried via set and get methods. This class is used as an input
 * to a channel object to query and set lighting parameters.
 *
 * Example code to query a lighting configuration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a lightingConfig instance to store our queried lighting configuration
 *     crl::multisense::lighting::Config lightingConfig;
 *
 *     //
 *     // Query the lighting configuration from the Channel instance
 *     crl::multisense::Status status = channel->getLightingConfig(lightingConfig);
 *
 *     //
 *     // Check to see if the lighting configuration query was successful
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query lighting configuration");
 *     }
 *
 *     //
 *     // Use the lighting configuration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set a lighting configuration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a lightingConfig instance to store our queried lighting configuration
 *     crl::multisense::lighting::Config lightingConfig;
 *
 *     crl::multisense::Status status;
 *
 *     //
 *     // Query the lighting configuration from the Channel instance
 *     status  = channel->getLightingConfig(lightingConfig);
 *
 *     //
 *     // Check to see if the lighting configuration query was successful
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query lighting configuration");
 *     }
 *
 *     //
 *     // Change the duty cycle for all light to the max setting and enable
 *     // light flashing
 *     lightingConfig.setDutyCycle(crl::multisense::lighting::MAX_DUTY_CYCLE);
 *     lightingConfig.setFlash(true);
 *
 *     //
 *     // Set the new lighting configuration
 *     status = channel->setLightingConfig(lightingConfig);
 *
 *     //
 *     // Check to see if the new lighting configuration was successfully
*      // received
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set lighting configuration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API Config {
public:

    /**
     * Turn on/off light flashing. During flashing lights are only on when the
     * image sensor is exposing. This significantly reduces the sensor power
     * consumption
     *
     * @param onOff A boolean flag to enable or disable flashing
     */

    void setFlash(bool onOff) { m_flashEnabled = onOff; };

    /**
     * Get the current lighting flash setting.
     *
     * @return A boolean flag corresponding to if flashing is enabled or disabled
     */

    bool getFlash()     const { return m_flashEnabled;  };

    /**
     * Set a sensors duty cycle in terms of percent for all the on-board lights
     *
     * @param percent The percent "on" to set all the on-board LED's [0.0, 100.0]
     *
     * @return If the given percent setting is valid
     */

    bool setDutyCycle(float percent) {
        if (percent < 0.0f || percent > MAX_DUTY_CYCLE)
            return false;

        std::fill(m_dutyCycle.begin(),
                  m_dutyCycle.end(),
                  percent);
        return true;
    };

    /**
     * Set a sensors duty cycle in terms of percent for a specific light based
     * off its index
     *
     * @param i The index of the light to configure
     *
     * @param percent The percent "on" to set all the on-board LED's [0.0, 100.0]
     *
     * @return If the given percent setting is valid
     */

    bool setDutyCycle(uint32_t i,
                      float    percent) {
        if (i >= MAX_LIGHTS ||
            percent < 0.0f || percent > MAX_DUTY_CYCLE)
            return false;

        m_dutyCycle[i] = percent;
        return true;
    };

    /**
     * Get the current duty cycle in terms of percent for a specific light
     *
     * @param i The index of the light to query
     *
     * @return The current duty cycle setting for the light corresponding to index
     * i
     */

    float getDutyCycle(uint32_t i) const {
        if (i >= MAX_LIGHTS)
            return 0.0f;
        return m_dutyCycle[i];
    };

    /**
    * Get the number of pulses of the light per a single exposure
    * This is used to trigger the light or output signal multiple times after a
    * single exposure. For values greater than 1, pulses will occur between the
    * exposures, not during. This can be used to leverage human persistence of
    * vision to make the light appear as though it is not flashing
    *
    * @return The current number of pulses
    */
    uint32_t getNumberOfPulses( ) const {
      return m_numberPulses;
    }

    /**
    * Set the number of pulses of the light within a single exposure
    * This is used to trigger the light or output signal multiple times after a
    * single exposure. For values greater than 1, pulses will occur between the
    * exposures, not during. This can be used to leverage human persistence of
    * vision to make the light appear as though it is not flashing
    *
    * @return True on success, False on failure
    */
    bool setNumberOfPulses(const uint32_t numPulses) {

      m_numberPulses = numPulses;
      return true;
    }

    /**
    * Get the startup time offset of the led in microseconds
    * The LED or output trigger is triggered this many microseconds before
    * the start of the image exposure
    *
    * @return The current led startup time
    */
    uint32_t getStartupTime( ) const {
      return m_lightStartupOffset_us;
    }

    /**
    * Set the transient startup time of the led, for better synchronization.
    * The LED or output trigger is triggered this many microseconds before
    * the start of the image exposure
    *
    * @param ledTransientResponse_us The led transient time.
    *
    * @return True on success, False on failure
    */
    bool setStartupTime(uint32_t ledTransientResponse_us) {

      m_lightStartupOffset_us = ledTransientResponse_us;
      return true;
    }

    /**
    * Get whether or not the LED pulse is inverted. True means the output
    * will be low during the exposure. False means the output will be high
    * during the exposure.
    *
    * @return True if the pulse is inverted
    */
    bool getInvertPulse( ) const {
        return m_invertPulse;
    }

    /**
    * Invert the output signal that drives lighting. True means the output
    * will be low during the exposure. False means the output will be high
    * during the exposure. (Only supported for firmware >=5.21)
    *
    * @param invert Whether or not to invert the pulse signal
    *
    * @return True on success, False on failure
    */
    bool setInvertPulse(const bool invert) {
        m_invertPulse = invert;
        return true;
    }
    /*
    * Enable a rolling shutter camera flash synchronization, this will Allow
    * an LED to flash with in sync with a rolling shutter imager, to reduce
    * the possibility of, seeing inconsistent lighting artifacts with rolling
    * shutter imagers.
    * Note: This feature is only available for Next Gen Stereo Cameras, with a
    * rolling shutter aux imager.
    *
    * @param enabled enable/disable the rolling shutter synchronization feature.
    *
    * @return True on success, False on failure
    */
    bool enableRollingShutterLedSynchronization(const bool enabled) {
      m_rollingShutterLedEnabled = enabled;
      return true;
    }

    /**
    * Get the setting of the rollingShutterSynchronization.
    * Note: This feature is only available for Next Gen Stereo Cameras, with a
    * rolling shutter imager.
    *
    * @return True if enabled, False if disabled
    */
    bool getRollingShutterLedSynchronizationStatus(void) const {
      return m_rollingShutterLedEnabled;
    }

    /**
     * Default constructor. Flashing is disabled and all lights are off
     */
    Config() : m_flashEnabled(false), m_dutyCycle(MAX_LIGHTS, -1.0f),
               m_numberPulses(1), m_lightStartupOffset_us(0), m_invertPulse(false),
               m_rollingShutterLedEnabled(false) {};

private:

    bool               m_flashEnabled;
    std::vector<float> m_dutyCycle;
    uint32_t           m_numberPulses;
    uint32_t           m_lightStartupOffset_us;
    bool               m_invertPulse;
    bool               m_rollingShutterLedEnabled;
};

/**
 * A external sensor status. This is only supported by external LED attachements
 * for S21 devices
 *
 * Example code to query the lighting sensor status :
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a lightingConfig instance to store our queried lighting configuration
 *     crl::multisense::lighting::SensorStatus lightingSensors;
 *
 *     //
 *     // Query the lighting configuration from the Channel instance
 *     crl::multisense::Status status = channel->getLightingSensorStatus(lightingSensors);
 *
 *     //
 *     // Check to see if the lighting sensor query was successful
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query lighting sensor status");
 *     }
 *
 *     //
 *     // Use the lighting sensor status...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API SensorStatus {
public:

    /**
     * This represents the percentage of light the ambient sensor currently sees.
     * External ambient sensors are only available on S21 units with the external
     * lighting attachement. This value ranges between 0 and 100
     */
    float ambientLightPercentage;

    SensorStatus() : ambientLightPercentage(100.0f) {};
};

} // namespace lighting

namespace pps {

/**
 * Class containing Header information for a PPS event. A network PPS event is
 * sent from the sensor immediately after the pulse on the OPTO-TX line.
 *
 * This will be passed to any callback, of type pps::Callback, subscribed to
 * PPS data.
 *
 * See crl::multisense::Channel::addIsolatedCallback
 */
class MULTISENSE_API Header : public HeaderBase {
public:

    /** The sensor time in nanoseconds when a given PPS event occurred  */
    int64_t sensorTime;

    /** The local time's seconds value */
    uint32_t timeSeconds;
    /** The local time's microseconds value */
    uint32_t timeMicroSeconds;
};

/**
 * Function pointer for receiving callbacks for PPS events
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

} // namespace pps

namespace imu {

/**
 * Class containing a single IMU sample. A sample can contain data
 * either accelerometer data, gyroscope data, or magnetometer data.
 */
class MULTISENSE_API Sample {
public:

    /** Typedef used to determine which data source a sample came from */
    typedef uint16_t Type;

    static CRL_CONSTEXPR Type Type_Accelerometer = 1;
    static CRL_CONSTEXPR Type Type_Gyroscope     = 2;
    static CRL_CONSTEXPR Type Type_Magnetometer  = 3;

    /** The type of data contained in the instance of imu::Sample */
    Type       type;
    /** The time seconds value corresponding to the specific sample */
    uint32_t   timeSeconds;
    /** The time microseconds value corresponding to the specific sample */
    uint32_t   timeMicroSeconds;

    /**
     * x data for each sample
     *
     * The units vary by source and can be
     * queried with crl::multisense::Channel::getImuInfo
     */
    float x;

    /**
     * y data for each sample
     *
     * The units vary by source and can be
     * queried with crl::multisense::Channel::getImuInfo
     */
    float y;

    /**
     * z data for each sample
     *
     * The units vary by source and can be
     * queried with crl::multisense::Channel::getImuInfo
     */
    float z;

    /**
     * A convenience function used for getting the time of the specific sample
     *
     * return The time corresponding to the sample as a double
     */

    double time() const {
        return (static_cast<double>(timeSeconds) +
                1e-6 * static_cast<double>(timeMicroSeconds));
    };
};

/**
 * Class containing Header information for an IMU callback.
 *
 * This will be passed to any callback, of type imu::Callback, subscribed to IMU
 * data
 *
 * See crl::multisense::Channel::addIsolatedCallback
 */
class MULTISENSE_API Header : public HeaderBase {
public:

    /** The sequence number for each header containing IMU samples. This
     * increments for each new header */
    uint32_t            sequence;
    /** A vector of samples from the sensor. IMU samples are aggregated on-board
     * the sensor and set at lower frequency to minimize the number of UDP
     * packets which need to be sent */
    std::vector<Sample> samples;
};

/**
 * Function pointer for receiving callbacks for IMU data
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

/**
 * Class containing detailed information for the IMU. A vector of Info classes
 * are returned by reference in crl::multisense::Channel::getImuInfo
 *
 * See http://docs.carnegierobotics.com/ for more info on the specific IMU
 * sensors used
 *
 * Example code to query IMU info:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a vector of IMU info instances to store information for
 *     // each IMU sensor
 *     std::vector<crl::multisense::imu::Info> imuInfoVect;
 *
 *     //
 *     // Create a uint32_t to store the maxSamplesPerMessage quantity returned
 *     // by reference from getImuInfo
 *     uint32_t maxSamplesPerMessage;
 *
 *     //
 *     // Query the IMU info from the Channel instance
 *     crl::multisense::Status status = channel->getImuInfo(maxSamplesPerMessage, imuInfoVect);
 *
 *     //
 *     // Check to see if the IMU info query was sucessful
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query imu info");
 *     }
 *
 *     //
 *     // Use the imu info...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API Info {
public:

    /**
     * Class containing a one valid IMU rate configuration
     */
    typedef struct {
        /** The sample rate of a given IMU source in Hz */
        float sampleRate;
        /** The bandwith cutoff for a given IMU source in Hz */
        float bandwidthCutoff;
    } RateEntry;

    /**
     * Class containing a one valid IMU range configuration
     */
    typedef struct {
        /** The range of valid sample readings for a IMU source. Value is +/-
         * units for a given IMU source */
        float range;
        /** The resolution setting for a given IMU source. In units per LSB */
        float resolution;
    } RangeEntry;

    /** The name of a specific IMU source */
    std::string             name;
    /** The device name for a specific IMU source */
    std::string             device;
    /** The units of for a specific IMU source */
    std::string             units;
    /** The various rates available for a specific IMU source */
    std::vector<RateEntry>  rates;
    /** The various ranges and resolutions available for a specific IMU source */
    std::vector<RangeEntry> ranges;
};

/**
 * Class used to store a specific IMU configuration.
 *
 * Example code to query a IMU configuration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create local variables to store the information returned by
 *     // reference from getImuConfig().
 *     std::vector<crl::multisense::imu::Config> imuConfigVect;
 *     uint32_t samplesPerMessage;
 *
 *     //
 *     // Query the IMU configuration from the Channel instance
 *     crl::multisense::Status status = channel->getImuConfig(samplesPerMessage, imuConfigVect);
 *
 *     //
 *     // Check to see if the IMU configuration query was successful
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query the imu configuration");
 *     }
 *
 *     //
 *     // Use the IMU configuration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set a IMU configuration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a vector of IMU configurations to store the queried IMU configuration
 *     std::vector<crl::multisense::imu::Config> imuConfigVect;
 *
 *     //
 *     // Create a uint32_t to store the samplesPerMessage quantity returned
 *     // by reference from getImuConfig
 *     uint32_t samplesPerMessage;
 *
 *     crl::multisense::Status status;
 *
 *     //
 *     // Query the IMU configuration from the Channel instance
 *     status = channel->getImuConfig(samplesPerMessage, imuConfigVect);
 *
 *     //
 *     // Check to see if the IMU configuration query was successful
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query the imu configuration");
 *     }
 *
 *     //
 *     // Enable streaming for the IMU device at index 0 and select the
 *     // rate corresponding to index 0 in crl::multisense::imu::Info::rates for
 *     // the corresponding IMU device
 *     imuConfigVect[0].enabled = true;
 *     imuConfigVect[0].rateTableIndex = 0;
 *
 *     //
 *     // Do not store these new settings in flash
 *     bool storeSettings = false;
 *
 *     //
 *     // Set the new IMU configuration. Keep the same samplesPerMessage setting
 *     // indicated by the 0 value
 *     crl::multisense::Status status = channel->setImuConfig(storeSettings, 0, imuConfigVect);
 *
 *     //
 *     // Check to see if the new IMU configuration was successfully received.
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set the imu configuration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 */
class MULTISENSE_API Config {
public:

    /** The name of a specific IMU source corresponding to
     * crl::multisense::imu::Info::name */
    std::string name;
    /** A boolean flag indicating whether the given IMU source is currently enabled */
    bool        enabled;
    /** The index into the rate table for a given IMU source specified in
     * crl::multisense::imu::Info::rates */
    uint32_t    rateTableIndex;
    /** The index into the range table for a given IMU source specified in
     * crl::multisense::imu::Info::ranges */
    uint32_t    rangeTableIndex;
};

} // namespace imu

namespace compressed_image {

class MULTISENSE_API Header : public HeaderBase {
public:

    /** DataSource corresponding to imageDataP*/
    DataSource  source;
    /** Bits per pixel in the image */
    uint32_t    bitsPerPixel;
    /** Compression codec */
    ImageCompressionCodec codec;
    /** Width of the image */
    uint32_t    width;
    /** Height of the image*/
    uint32_t    height;
    /** Unique ID used to describe an image. FrameIds increase sequentally from the device */
    int64_t     frameId;
    /** The time seconds value corresponding to when  the image was captured*/
    uint32_t    timeSeconds;
    /** The time microseconds value corresponding to when the image was captured*/
    uint32_t    timeMicroSeconds;

    /** The image exposure time in microseconds*/
    uint32_t    exposure;
    /** The imager gain the image was captured with */
    float       gain;
    /** The number of frames per second currently streaming from the device */
    float       framesPerSecond;
    /** The length of the image data stored in imageDataP */
    uint32_t    imageLength;
    /** A pointer to the compressed image data */
    const void *imageDataP;

    /**
     * Default Constructor
     */
    Header()
        : source(Source_Unknown) {};

    /**
     * Member function used to determine if the data contained in the header
     * is contained in a specific image mask
     */
    virtual bool inMask(DataSource mask) { return (mask & source) != 0;};
};

/**
 * Function pointer for receiving callbacks for compressed image data
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

} // namespace compressed_image

namespace ground_surface {

/**
 * Class containing Header information for a Ground Surface Spline callback.
 *
 * See crl::multisense::Channel::addIsolatedCallback
 */
class MULTISENSE_API Header : public HeaderBase {
public:
    /** Unique ID used to describe an image. FrameIds increase sequentally from the device */
    int64_t     frameId;
    /** Trigger time of the disparity image which was used to generate the spline */
    int64_t     timestamp;
    /** Success flag for B-Spline fitting process — no control points will be available if this is false **/
    uint8_t     success;

    /** Bits per pixel in the dynamically-sized control points array */
    uint32_t    controlPointsBitsPerPixel;
    /** Width of the dynamically-sized control points array */
    uint32_t    controlPointsWidth;
    /** Height of the dynamically-sized control points array */
    uint32_t    controlPointsHeight;
    /** A pointer to the dynamically-sized control points array data */
    const void *controlPointsImageDataP;

    /** X,Z cell origin of the spline fitting algorithm in meters */
    float xzCellOrigin[2];
    /** Size of the X,Z plane containing the spline fit in meters */
    float xzCellSize[2];
    /** X,Z limit to the spline fitting area in meters */
    float xzLimit[2];
    /** Min and max limit to the spline fitting angle in radians, for visualization purposes */
    float minMaxAzimuthAngle[2];

    /** Camera extrinsics that were used in the ground surface fitting operation
     *  Order of parameters is x, y, z in meters then rz, ry, rz in radians */
    float extrinsics[6];

    /** Parameters for the quadratic data transformation prior to spline fitting
     *      (ax^2 + by^2 + cxy + dx + ey + f) */
    float quadraticParams[6];
};

/**
 * Function pointer for receiving callbacks for Ground Surface Spline data
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

} // namespace ground_surface

namespace apriltag {

/**
 * Class containing Header information for a apriltag callback.
 *
 * See crl::multisense::Channel::addIsolatedCallback
 */
class MULTISENSE_API Header : public HeaderBase {
public:
    struct ApriltagDetection {
        /** The family of the tag */
        std::string family;
        /** The ID of the tag */
        uint32_t id;
        /** The hamming distance between the detection and the real code */
        uint8_t hamming;
        /** The quality/confidence of the binary decoding process
         * average difference between intensity of data bit vs decision thresh.
         * Higher is better. Only useful for small tags */
        float decisionMargin;
        /** The 3x3 homography matrix describing the projection from an
         * "ideal" tag (with corners at (-1,1), (1,1), (1,-1), and (-1,
         * -1)) to pixels in the image */
        double tagToImageHomography[3][3];
        /** The 2D position of the origin of the tag in the image */
        double center[2];
        /** The 4 tag corner pixel locations in the order:
         * point 0: [-squareLength / 2, squareLength / 2]
         * point 1: [ squareLength / 2, squareLength / 2]
         * point 2: [ squareLength / 2, -squareLength / 2]
         * point 3: [-squareLength / 2, -squareLength / 2] */
        double corners[4][2];
    };

    /** The frame ID of the image that the apriltags were detected on */
    int64_t     frameId;
    /** The frame timestamp (nanoseconds) of the image that the apriltags were detected on */
    int64_t     timestamp;
    /** The image source that the apriltags were detected on */
    std::string imageSource;
    /** Success flag to indicate whether for the apriltag algorithm ran successfully */
    uint8_t     success;
    /** The number of apriltags that were detected */
    uint32_t    numDetections;
    /** Apriltag detections */
    std::vector<ApriltagDetection> detections;
};

/**
 * Function pointer for receiving callbacks for apriltag data
 */
typedef void (*Callback)(const Header& header,
                         void         *userDataP);

} // namespace apriltag

namespace feature_detector {


  /** The recommended maximum number of features for full resolution camera operation */
  static CRL_CONSTEXPR int RECOMMENDED_MAX_FEATURES_FULL_RES    = 5000;
  /** The recommended maximum number of features for quarter resolution camera operation */
  static CRL_CONSTEXPR int RECOMMENDED_MAX_FEATURES_QUARTER_RES = 1500;


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

  class MULTISENSE_API Header : public HeaderBase {
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
      std::vector<Feature> features;
      std::vector<Descriptor> descriptors;
  };

  /**
   * Function pointer for receiving callbacks for feature_detector data
   */
  typedef void (*Callback)(const Header& header,
                           void         *userDataP);

} // namespace feature_detector


namespace system {

/**
 * Class used query the device modes for a given sensor.
 *
 * Example code to query all the available device modes for a sensor.
 * \code{.cpp}
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     std::vector<crl::multisense::system::DeviceMode> deviceModeVect;
 *
 *     crl::multisense::Status status = channel->getDeviceModes(deviceModeVect));
 *
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query device modes info");
 *     }
 *
 *     //
 *     // Use the device modes...
 *
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API DeviceMode {
public:

    /** The image width configuration for a given device mode */
    uint32_t   width;
    /** The image height configuration for a given device mode */
    uint32_t   height;
    /** A listing of all the data sources available for a specific device mode */
    DataSource supportedDataSources;
    /** The number of valid disparities for a given device mode */
    int32_t    disparities;

    /**
     * Constructor
     *
     * @param w The device mode width. Default value: 0
     *
     * @param h The device mode height. Default value: 0
     *
     * @param d The available data sources. Default value: 0
     *
     * @param s The number of valid disparities. Default value: -1
     */
    DeviceMode(uint32_t   w=0,
               uint32_t   h=0,
               DataSource d=0,
               int32_t    s=-1) :
        width(w),
        height(h),
        supportedDataSources(d),
        disparities(s) {};
};

/**
 * Class containing version info for a specific sensor.
 *
 * Example code to query a sensors version info
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a VersionInfo instance to store the sensors version info
 *     crl::multisense::system::VersionInfo versionInfo;
 *
 *     //
 *     // Query the version info from the Channel instance
 *     crl::multisense::Status status = channel->getVersionInfo(versionInfo));
 *
 *     //
 *     // Check to see if the version info was queried successfully
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query sensor version info");
 *     }
 *
 *     //
 *     // Use the version info ...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 */
class MULTISENSE_API VersionInfo {
public:

    /** The build date of libMultiSense */
    std::string apiBuildDate;
    /** The version of libMultiSense */
    VersionType apiVersion;

    /** The build date of the sensor firmware */
    std::string sensorFirmwareBuildDate;
    /** The version type of the sensor firmware */
    VersionType sensorFirmwareVersion;

    /** The hardware version of the sensor */
    uint64_t    sensorHardwareVersion;
    /** The hardware magic number */
    uint64_t    sensorHardwareMagic;
    /** The FPGA DNA */
    uint64_t    sensorFpgaDna;

    /**
     * Default constructor which initialize all values to 0
     */
    VersionInfo() :
        apiVersion(0),
        sensorFirmwareVersion(0),
        sensorHardwareVersion(0),
        sensorHardwareMagic(0),
        sensorFpgaDna(0) {};
};

/**
 * Class used to store PCB information for the various circuit boards in
 * a sensor
 */
class MULTISENSE_API PcbInfo {
public:

    /** The name of a PCB */
    std::string name;
    /** The revision number of a PCB */
    uint32_t    revision;

    /**
     * Default constructor
     */
    PcbInfo() : revision(0) {};
};

/**
 * Class used to store device information specific to a sensor.
 *
 * Example code to query device information from a sensor:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of Device info to store the sensors device information
 *     crl::multisense::system::DeviceInfo deviceInfo;
 *
 *     //
 *     // Query the device information from the Channel instance
 *     crl::multisense::Status status = channel->getDeviceInfo(deviceInfo));
 *
 *     //
 *     // Check to see if the device information query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query sensor device info");
 *     }
 *
 *     //
 *     // Use the device information...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Setting sensor device info is not publicly supported.
 */
class MULTISENSE_API DeviceInfo {
public:

    /** The maximum number of PCBs in a device */
    static CRL_CONSTEXPR uint8_t MAX_PCBS                   = 8;

    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_SL                  = 1;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S7                  = 2;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S                   = HARDWARE_REV_MULTISENSE_S7; // alias for backward source compatibility
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_M                   = 3;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S7S                 = 4;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S21                 = 5;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_ST21                = 6;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_C6S2_S27            = 7;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S30                 = 8;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_S7AR                = 9;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_KS21                = 10;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_MONOCAM             = 11;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_REMOTE_HEAD_VPB     = 12;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_REMOTE_HEAD_STEREO  = 13;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_REMOTE_HEAD_MONOCAM = 14;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_KS21_SILVER         = 15;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_ST25                = 16;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MULTISENSE_KS21i               = 17;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_BCAM                           = 100;
    static CRL_CONSTEXPR uint32_t HARDWARE_REV_MONO                           = 101;

    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_NONE           = 0;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV2000_GREY   = 1;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV2000_COLOR  = 2;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV4000_GREY   = 3;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_CMV4000_COLOR  = 4;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_FLIR_TAU2      = 7;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_IMX104_COLOR   = 100;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_AR0234_GREY    = 200;
    static CRL_CONSTEXPR uint32_t IMAGER_TYPE_AR0239_COLOR   = 202;

    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_NONE = 0;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_INTERNAL = 1;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_EXTERNAL = 2;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_PATTERN_PROJECTOR = 3;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_OUTPUT_TRIGGER = 4;
    static CRL_CONSTEXPR uint32_t LIGHTING_TYPE_PATTERN_PROJECTOR_AND_OUTPUT_TRIGGER = 5;

    /** The name of a given device */
    std::string name;
    /** The date the device was manufactured */
    std::string buildDate;
    /** The serial number of the device */
    std::string serialNumber;
    /** The hardware revision of the given sensor */
    uint32_t    hardwareRevision;

    /** The information for all the PCBs in the device */
    std::vector<PcbInfo> pcbs;

    /** The name of the sensor's imager */
    std::string imagerName;
    /** The type of the sensor's imager */
    uint32_t    imagerType;
    /** The maximum width of the sensor's imager */
    uint32_t    imagerWidth;
    /** The maximum height of the sensor's imager */
    uint32_t    imagerHeight;

    /** The name of the sensor's lens */
    std::string lensName;
    /** The type of the sensor's lens */
    uint32_t    lensType;
    /** The nominal sensor baseline in meters */
    float       nominalBaseline;
    /** The nominal focal length for the lens in meters */
    float       nominalFocalLength;
    /** The nominal relative aperature for the sensor. i.e. the f-stop */
    float       nominalRelativeAperture;

    /** The lighting type supported by the sensor */
    uint32_t    lightingType;
    /** The number of lights supported by the sensor */
    uint32_t    numberOfLights;

    /** The name of the sensor's laser */
    std::string laserName;
    /** The type of the sensor's laser */
    uint32_t    laserType;

    /** The name of the sensor's motor */
    std::string motorName;
    /** The type of the sensor's motor */
    uint32_t    motorType;
    /** The gear reduction for the sensor's laser assembly */
    float       motorGearReduction;

    /**
     * Default constructor
     */
    DeviceInfo() :
        hardwareRevision(0),
        imagerType(0),
        imagerWidth(0),
        imagerHeight(0),
        lensType(0),
        nominalBaseline(0.0),
        nominalFocalLength(0.0),
        nominalRelativeAperture(0.0),
        lightingType(0),
        numberOfLights(0),
        laserType(0),
        motorType(0),
        motorGearReduction(0.0) {};
};

/**
 * Class containing the network configuration for a specific sensor.
 *
 * Example code to query a sensor's network configuration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of NetworkConfig to store the sensor's network configuration
 *     crl::multisense::system::NetworkConfig networkConfig;
 *
 *     //
 *     // Query the network configuration from the Channel instance
 *     crl::multisense::Status status = channel->getNetworkConfig(networkConfig));
 *
 *     //
 *     // Check to see if the network configuration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query sensor's network configuration");
 *     }
 *
 *     //
 *     // Use the network configuration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set a sensor's network configuration:
 ** \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a new instance of a Network configuration with the new desired
 *     // network configuration
 *     crl::multisense::system::NetworkConfig networkConfig("10.66.171.22",
 *                                                          "10.66.171.1",
 *                                                          "255.255.255.0");;
 *
 *     //
 *     // Send the new network configuration to the device
 *     crl::multisense::Status status = channel->setNetworkConfig(networkConfig));
 *
 *     //
 *     // Check to see if the new network configuration was received
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set the sensor's network configuration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API NetworkConfig {
public:

    /** An Ipv4 address corresponding to a sensor */
    std::string ipv4Address;
    /** An Ipv4 gateway corresponding to a sensor */
    std::string ipv4Gateway;
    /** An Ipv4 netmask corresponding to a sensor */
    std::string ipv4Netmask;

    /**
     * Default constructor with the sensor factory default IP configuration
     */
    NetworkConfig() :
        ipv4Address("10.66.171.21"),
        ipv4Gateway("10.66.171.1"),
        ipv4Netmask("255.255.240.0") {};

    /**
     * Constructor to initialize the Ipv4 parameters
     *
     * @param a A Ipv4 address
     *
     * @param g A Ipv4 gateway
     *
     * @param n A Ipv4 netmask
     */
    NetworkConfig(const std::string& a,
                  const std::string& g,
                  const std::string& n) :
        ipv4Address(a),
        ipv4Gateway(g),
        ipv4Netmask(n) {};
};

/**
 * Class containing status information for a particular device. This
 * should be queried in a loop timed at 1Hz
 *
 * Example code to query a single sensor's status:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of StatusMessage to store the sensor's status
 *     crl::multisense::system::StatusMessage statusMessage;
 *
 *     //
 *     // Query the network configuration from the Channel instance
 *     crl::multisense::Status status = channel->getDeviceStatus(statusMessage);
 *
 *     //
 *     // Check to see if the network configuration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query sensor's status");
 *     }
 *
 *     //
 *     // Use the device status...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API StatusMessage {
    public:

        /** The system uptime of the MultiSense in seconds.
         * True corresonds to healthy */
        double uptime;

        /** A boolean flag indicating if the overall system status is good.
         * True corresonds to healthy */
        bool systemOk;

        /** A boolean flag indicating if the laser is functioning.
         * True corresonds to healthy */
        bool laserOk;

        /** A boolean flag indicating if the laser motor controller is functioning.
         * True corresonds to healthy */
        bool laserMotorOk;

        /** A boolean flag indicating if the imagers are functioning.
         * True corresonds to healthy */
        bool camerasOk;

        /** A boolean flag indicating if the imu is functioning.
         * True corresonds to healthy */
        bool imuOk;

        /** A boolean flag indicating if the external LEDs are OK. This flag
         * will only be true if external LEDs are present */
        bool externalLedsOk;

        /** A boolean indicating if the processing pipeline is ok */
        bool processingPipelineOk;

        /** The temperature of the internal switching mode power supply.
         * Temperature is is Celsius */
        float powerSupplyTemperature;

        /** The temperature of the FPGA. Temperature is is Celsius */
        float fpgaTemperature;

        /** The temperature of the left imager. Temperature is is Celsius */
        float leftImagerTemperature;

        /** The temperature of the right imager. Temperature is is Celsius */
        float rightImagerTemperature;

        /** The input voltage supplied to the MultiSense. Value is in Volts */
        float inputVoltage;

        /** The current drawn from the input power supply by the MultiSense. Value
         * is in Amperes */
        float inputCurrent;

        /** The power consumed by the FPGA. Value is in Watts */
        float fpgaPower;

        /** The power consumed by the MicroBlaze CPU. Value is in Watts */
        float logicPower;

        /** The power consumed by the imager chips. Value is in Watts */
        float imagerPower;

        /** Default constructor for a single StatusMessage object */
        StatusMessage():
            uptime(0.),
            systemOk(false),
            laserOk(false),
            laserMotorOk(false),
            camerasOk(false),
            imuOk(false),
            externalLedsOk(false),
            processingPipelineOk(false),
            powerSupplyTemperature(0.),
            fpgaTemperature(0.),
            leftImagerTemperature(0.),
            rightImagerTemperature(0.),
            inputVoltage(0.),
            inputCurrent(0.),
            fpgaPower(0.),
            logicPower(0.),
            imagerPower(0.) {};
};

/**
 * A external calibration associated with the MultiSense. This is user defined
 * non-volatile storage so the location of the MultiSense can be stored dynamically
 * on the sensor. This can be used to store the mounting location of the sensor
 * relative to a base coordinate frame. This is not used internally by the MultiSense
 * or the ROS driver.
 *
 * Example code to query a devices's external calibration:
 * \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of ExternalCalibration to store the device's imager
      // calibration
 *     crl::multisense::system::ExternalCalibration externalCalibration;
 *
 *     //
 *     // Query the imager calibration from the Channel instance
 *     crl::multisense::Status status = channel->getExternalCalibration(externalCalibration));
 *
 *     //
 *     // Check to see if the network configuration query succeeded
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to query device's external calibration");
 *     }
 *
 *     //
 *     // Use the external calibration...
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 *
 * Example code to set a devices external calibration:
 ** \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of ExternalCalibration to store the device's imager
 *     // calibration
 *     crl::multisense::system::ExternalCalibration externalCalibration;
 *
 *     //
 *     // Set the external calibration values
 *     externalCalibration.x = 0.1;
 *     externalCalibration.y = 0.2;
 *     externalCalibration.z = 0.3;
 *     externalCalibration.roll = 2.5;
 *     externalCalibration.pitch = 3.67;
 *     externalCalibration.yaw = 1.2;
 *
 *     //
 *     // Send the new external calibration to the device
 *     crl::multisense::Status status = channel->setExternalCalibration(externalCalibration));
 *
 *     //
 *     // Check to see if the new network configuration was received
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set the devices's imager calibration");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API ExternalCalibration {
    public:

        /** The external x translation of the MultiSense in meters */
        float x;

        /** The external y translation of the MultiSense in meters */
        float y;

        /** The external z translation of the MultiSense in meters */
        float z;

        /** The external roll translation of the MultiSense in degrees */
        float roll;

        /** The external pitch translation of the MultiSense in degrees */
        float pitch;

        /** The external yaw translation of the MultiSense in degrees */
        float yaw;

        /** Default constructor. By default this transform is Identity */
        ExternalCalibration():
            x(0.),
            y(0.),
            z(0.),
            roll(0.),
            pitch(0.),
            yaw(0.) {};
};

/**
 * Class containing parameters for the ground surface modeling and obstacle detection
 * application which may be running on the specifically commissioned MultiSenses.
 *
 * Example code to set a device's ground surface parameters:
 ** \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of GroundSurfaceParams to store the device's params
 *     crl::multisense::system::GroundSurfaceParams params;
 *
 *     //
 *     // Set the parameter values
 *     params.ground_surface_number_of_levels_x = 4;
 *     params.ground_surface_number_of_levels_z = 4;
 *     params.ground_surface_base_model = 1;
 *     params.ground_surface_pointcloud_grid_size = 0.5;
 *     params.ground_surface_min_points_per_grid = 10;
 *     params.ground_surface_pointcloud_decimation = 1;
 *     params.ground_surface_pointcloud_max_range_m = 30.0;
 *     params.ground_surface_pointcloud_min_range_m = 0.5;
 *     params.ground_surface_pointcloud_max_width_m = 25.0;
 *     params.ground_surface_pointcloud_min_width_ = -25.0;
 *     params.ground_surface_pointcloud_max_height_m = 10.0;
 *     params.ground_surface_pointcloud_min_height_ = -10.0;
 *     params.ground_surface_obstacle_height_thresh_m = 2.0;
 *     params.ground_surface_obstacle_percentage_thresh = 0.5;
 *     params.ground_surface_max_fitting_iterations = 10;
 *     params.ground_surface_adjacent_cell_search_size_m = 1.5;
 *
 *     //
 *     // Send the new external calibration to the device
 *     crl::multisense::Status status = channel->setGroundSurfaceParams(params));
 *
 *     //
 *     // Check to see if the new network configuration was received
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set the devices's ground surface params");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API GroundSurfaceParams {
    public:

        /** This argument specifies how many levels of spline interpolation are to be performed in the x dimension
         * (i.e. along the left to right horizontal dimension in front of the camera' FOV). The ultimate size of
         * the grid of spline control points is approximately 2^number_of_levels by 2^number_of_levels. Keep in mind
         * that each interpolated point draws from a 4x4 support region in the grid of spline control points. */
        int ground_surface_number_of_levels_x;

        /** This argument specifies how many levels of spline interpolation are to be performed in the z dimension
         * (i.e. along the optical axis, or depth towards/away from the camera). The ultimate size of the grid of
         * spline control points is approximately 2^number_of_levels by 2^number_of_levels. Keep in mind that each
         * interpolated point draws from a 4x4 support region in the grid of spline control points. */
        int ground_surface_number_of_levels_z;

        /** The model to apply to the raw pointcloud data before modeling with a B-Spline. The Mean model is a good
         * default as simplyassumes the world is a plane about the mean height of the pointcloud (after applying the
         * external transform). If the mounting height of the camera is set appropriately in the external transform,
         * and the world is generally flat, then Zero is a good choice. TheQuadratic model is useful when the extent
         * of the pointcloud is trending upwards or downwards (i.e. while using forwards-facing camera mounted on a
         * vehicle driving through a valley). */
        int ground_surface_base_model;

        /** This is the size of grid dimension that the poiontcloud is binned into along the X/Z plane. A larger
         * grid size means a means a coarser binned pointcloud and faster processing times, while a smaller grid
         * size means a finer binned pointcloud. */
        float ground_surface_pointcloud_grid_size;

        /** This is the minimum number of pointcloud points which ust be within a binned grid cell in oder for that
         * cell to be used in the spline computation. This threshold helps to reduce the effect of spurious 3D points
         * on spline modeling. */
        int ground_surface_min_points_per_grid;

        /** The decimation factor for the disparity image when generating the pointcloud */
        int ground_surface_pointcloud_decimation;

        /** The max pointcloud range (along the z dimension / optical axis) after applying external transform,
         * a useful parameter to remove noisy points far from the camera */
        float ground_surface_pointcloud_max_range_m;

        /** The min pointcloud range (along the z dimension / optical axis) after applying external transform,
         * a useful parameter to remove noisy points close to the camera */
        float ground_surface_pointcloud_min_range_m;

        /** The max pointcloud width (along the x dimension) after applying external transform */
        float ground_surface_pointcloud_max_width_m;

        /** The min pointcloud width (along the x dimension) after applying external transform */
        float ground_surface_pointcloud_min_width_m;

        /** The max pointcloud height (along the y dimension) after applying external transform, a useful
         * parameter to remove noisy points in the sky */
        float ground_surface_pointcloud_max_height_m;

        /** The min pointcloud height (along the y dimension) after applying external transform, a useful
         * parameter to remove noisy points in the ground */
        float ground_surface_pointcloud_min_height_m;

        /** This is the height threshold, in meters, that a cluster of points must be above the initial ground
         * surface model fit in order to be classified as an obstacle. */
        float ground_surface_obstacle_height_thresh_m;

        /** The percentage of points in a cell cluster that must be above the height threshold in order to classify
         * the cell as an obstacle. For example, obstacle_percentage_thresh=0.5 requires at least half of the points
         * in the binned cell to be above the height threshold. */
        float ground_surface_obstacle_percentage_thresh;

        /** The maximum number of iterations to undertake in the spline fit and obstacle detection loop. This loop
         * will exit early if no new obstacle cells are found from one interation to the next. */
        int ground_surface_max_fitting_iterations;

        /** The size of the neighborhood to search around an obstacle cell for the lowest/minimum cell centroid
         * height during the iterative spline fit and obstacle detection loop. */
        float ground_surface_adjacent_cell_search_size_m;

        /** Default constructor */
        GroundSurfaceParams():
            ground_surface_number_of_levels_x(4),
            ground_surface_number_of_levels_z(4),
            ground_surface_base_model(1),
            ground_surface_pointcloud_grid_size(0.5),
            ground_surface_min_points_per_grid(10),
            ground_surface_pointcloud_decimation(1),
            ground_surface_pointcloud_max_range_m(30.0),
            ground_surface_pointcloud_min_range_m(0.5),
            ground_surface_pointcloud_max_width_m(25.0),
            ground_surface_pointcloud_min_width_m(-25.0),
            ground_surface_pointcloud_max_height_m(10.0),
            ground_surface_pointcloud_min_height_m(-10.0),
            ground_surface_obstacle_height_thresh_m(2.0),
            ground_surface_obstacle_percentage_thresh(0.5),
            ground_surface_max_fitting_iterations(10),
            ground_surface_adjacent_cell_search_size_m(1.5) {};
};

/**
 * Class containing parameters for the apriltag fiduciual detection algorithm
 * application which may be running on the specifically commissioned MultiSenses.
 *
 * Example code to set a device's apriltag parameters:
 ** \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     //
 *     // Create a instance of ApriltagParams to store the device's params
 *     crl::multisense::system::ApriltagParams params;
 *
 *     //
 *     // Set the parameter values
 *     params.family = "tagStandard52h13";
 *     params.max_hamming = 0;
 *     params.quad_detection_blur_sigma = 0.75;
 *     params.quad_detection_decimate = 1.0;
 *     params.min_border_width = 5;
 *     params.refine_quad_edges = false;
 *     params.decode_sharpening = 0.25;
 *
 *     //
 *     // Send the new external calibration to the device
 *     crl::multisense::Status status = channel->setApriltagParams(params));
 *
 *     //
 *     // Check to see if the new network configuration was received
 *     if(crl::multisense::Status_Ok != status) {
 *          throw std::runtime_error("Unable to set the devices's apriltag params");
 *     }
 *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */
class MULTISENSE_API ApriltagParams {
    public:
        /** Apriltag family to detect */
        std::string family;

        /** The maximum hamming correction that will be applied while detecting codes */
        uint8_t max_hamming;

        /** Sigma of the Gaussian blur applied to the image before quad_detection
         * Specified in full resolution pixels. Kernel size = 4*sigma, rounded up to
         * the next odd number. (<0.5 results in no blur) */
        double quad_detection_blur_sigma;

        /** Amount to decimate image before detecting quads. Values < 1.0
         *(0.5 decimation reduces height/width by half) */
        double quad_detection_decimate;

        /** Minimum border width that can be considered a valid tag. Used to filter
         * contours based on area and perimeter. Units are in undecimated image pixels.
         * Increasing the value will speed up the detector and may reduce false detection rates
         * at the expense of reducing ability to detect small tags.
         * NOTE: If this value is smaller than the smallest tag border, detector will override
         * with the minimum border width out of the active families in the detector.
         *
         *        Family border widths for reference:
         *            tag16h5 : 6 px
         *            tag25h9 : 7 px
         *            tag36h11 : 8 px
         *            tagCircle21h7 : 5 px
         *            tagCircle49h12 : 5 px
         *            tagCustom48h12 : 6 px
         *            tagStandard41h12 : 5 px
         *            tagStandard52h13 : 6 px
         */
        size_t min_border_width;

        /** Whether or not to refine the edges before attempting to decode */
        bool refine_quad_edges;

        /** How much to sharpen the quad before sampling the pixels. After the homography
         * turns the sampled pixels into a perfect square image representing the tag bits,
         * the sharpening is applied bring out the potentially soft edges. 0 to turn off,
         * large values are stronger sharpening, recommended to stay below 1 */
        double decode_sharpening;

        /** Default constructor */
        ApriltagParams():
            family("tagStandard52h13"),
            max_hamming(0),
            quad_detection_blur_sigma(0.75),
            quad_detection_decimate(1.0),
            min_border_width(5),
            refine_quad_edges(false),
            decode_sharpening(0.25) {};
};

/**
 * Example code showing usage of the onboard feature detector.
 * Can also reference FeatureDetectorUtility.cc
 *
 * Example code to set a device's feature detection parameters:
 ** \code{.cpp}
 *     //
 *     // Instantiate a channel connecting to a sensor at the factory default
 *     // IP address
 *     crl::multisense::Channel* channel;
 *     channel = crl::multisense::Channel::Create("10.66.171.21");
 *
 *     channel->setMtu(1500);
 *
 *     FeatureDetectorConfig fcfg;
 *
 *     status = channelP->getFeatureDetectorConfig(fcfg);
 *     if (Status_Ok != status) {
 *           std::cerr << "Failed to get feature detector config: " << Channel::statusString(status) << std::endl;
 *             goto clean_out;
 *     }
 *
 *     if (quarter_res)
 *         fcfg.setNumberOfFeatures(1500);
 *     else
 *         fcfg.setNumberOfFeatures(5000);
 *
 *     fcfg.setGrouping(true);
 *     fcfg.setMotion(1);
 *
 *     status = channelP->setFeatureDetectorConfig(fcfg);
 *     if (Status_Ok != status) {
 *       std::cerr << "Failed to set feature detector config\n";
 *         goto clean_out;
 *     }
 *
 *     //
 *     // Add Image Callback
 *     channelP->addIsolatedCallback(imageCallback, Source_Luma_Left|Source_Luma_Right, &userData);
 *     //
 *     // Add Feature Callback
 *     channelP->addIsolatedCallback(featureDetectorCallback, &userData);
 *
 *     //
 *     // Start streaming
 *     status = channelP->startStreams((operatingMode.supportedDataSources & Source_Luma_Left)  |
 *                                     (operatingMode.supportedDataSources & Source_Luma_Right) |
 *                                     (operatingMode.supportedDataSources & Source_Feature_Left)|
 *                                     (operatingMode.supportedDataSources & Source_Feature_Right));
 *     if (Status_Ok != status) {
 *         std::cerr << "Failed to start streams: " << Channel::statusString(status) << std::endl;
 *         goto clean_out;
 *     } *
 *     //
 *     // Destroy the channel instance
 *     crl::multisense::Channel::Destroy(channel);
 * \endcode
 */

class MULTISENSE_API FeatureDetectorConfig {

    private:

        /**
         * numberOfFeatures
         * The maximum features to be searched for in one image.
         *
         * Current recommended settings.
         * Full    Resolution: 5000 Features @5FPS
         * Quarter Resolution: 1500 Features @15FPS
         */
        uint32_t m_numberOfFeatures;

        /**
         * grouping
         * Enable/Disable the grouping feature in feaure detection.
         * Grouping adds scale invariance to ORB features, by detecting the same
         * feature in multiple octaves, and grouping the feature.
         * Grouping reduces redundant features and eliminates the need to keep
         * track of features referencing  the same corner.
         * When grouping is enabled, the user should expect less features than
         * descriptors, which should result in computationally easier feature matching,
         * between consecutive frames.
         * Although grouping does come at a slightly reduced framerate, it is
         * recommended and verified at the recommended settings.
         */
        bool m_grouping;

        /**
         * motion
         * Enable / disable motion detection in the feature detector.
         * When enabled, you can check the averageXMotion, averageYMotion and
         * motionStatus of the feaure_detector::header.
         * averageXMotion and averageYMotion == 65535 corresponds to a failed
         * motion detection for that feature frame.
         */
        uint32_t m_motion;

    public:
        /**
         * Query the maximum number of features applied to the camera feature detector
         *
         * @return Return the current maximum number of features
         */
        uint32_t numberOfFeatures() const { return m_numberOfFeatures; };

        /**
         * Query the status of the feature detector feature grouping
         *
         * @return Return the current feature grouping status
         */
        bool grouping() const { return m_grouping; };

        /**
         * Query the status of the feature detector motion detection
         *
         * @return Return the current feature detector motion detection status
         */
        bool motion() const { return m_motion; };

        /**
         * Set the maximum number of features applied to the camera feature detector.
         * Current recommended settings.
         * Full    Resolution: 5000 Features @5FPS
         * Quarter Resolution: 1500 Features @15FPS
         *
         * @param numberOfFeatures The maximum number of features.
         */

        void setNumberOfFeatures(const uint32_t &numberOfFeatures)    {

            if (numberOfFeatures > feature_detector::RECOMMENDED_MAX_FEATURES_FULL_RES)
            {
                std::cout << "WARNING: The number of features requested is above recommended level!" << '\n';
                std::cout << "If a performance impact is noticed reduce number of features and/or framerate of camera" << '\n';
                std::cout << "The recommended maximum camera settings when using the feature detector is:" << '\n';
                std::cout << "Quarter Res: 15FPS and 1500 Features" << '\n';
                std::cout << "Full    Res:  5FPS and 5000 Features" << '\n';
            }

            m_numberOfFeatures = numberOfFeatures;

        };

        /**
         * Set the feature grouping capability the feature detector
         *
         * @param g The feature grouping to apply to this camera
         */
        void setGrouping(const bool &g)    {
            m_grouping = g;
        }

        /**
         * Set the feature motion detection capability of the feature detector
         * Functions to enable motion detection on Octave 3
         *
         *
         * @param m The feature detector motion detector.
         */
        void setMotion(const uint32_t &m)    {
            m_motion = m;
        }

        /** Default constructor */
        FeatureDetectorConfig():
            m_numberOfFeatures(feature_detector::RECOMMENDED_MAX_FEATURES_QUARTER_RES),
            m_grouping(true),
            m_motion(1)
        {};
};

/**
 * PTP status data associated with a specific stamped MultiSense message
 */
class MULTISENSE_API PtpStatus {
    public:
        /** Status of grandmaster clock; 1 if synchronized to nonlocal GM
         * OR if nonlocal GM was present any time during current boot. 0 Otherwise*/
        uint8_t gm_present;

        /** Hex ID of grandmaster clock. */
        uint8_t gm_id[8];

        /** Offset of camera PHC to PTP grandmaster clock in nanosec */
        int64_t gm_offset;

        /** Estimated delay of syncronization messages from master in nanosec */
        int64_t path_delay;

        /** Number of network hops from GM to local clock */
        uint16_t steps_removed;

        /** Default constructor for a single PtpStatus object */
        PtpStatus():
            gm_present(0),
            gm_offset(0),
            path_delay(0),
            steps_removed(0)
        {
            memset(gm_id, 0, sizeof(gm_id));
        };
};

/**
 * A struct for storing statistics for a channel object
 */
struct ChannelStatistics
{
    ChannelStatistics():
        numMissedHeaders(0),
        numDroppedAssemblers(0),
        numImageMetaData(0),
        numDispatchedImage(0),
        numDispatchedLidar(0),
        numDispatchedPps(0),
        numDispatchedImu(0),
        numDispatchedCompressedImage(0),
        numDispatchedGroundSurfaceSpline(0),
        numDispatchedAprilTagDetections(0)
    {
    };

    ~ChannelStatistics()
    {
    };

    //
    // The total number of sequence ids observed where headers appeared to have
    // been missed
    std::size_t numMissedHeaders;

    //
    // The number of UDP assemblers that were dropped from the depth cache
    // while awaiting assembly
    std::size_t numDroppedAssemblers;

    //
    // The number of image metadata messages that were received
    std::size_t numImageMetaData;

    //
    // The number of images that were dispatched
    std::size_t numDispatchedImage;

    //
    // The number of lidar scans that were dispatched
    std::size_t numDispatchedLidar;

    //
    // The number of dispatched PPS messages
    std::size_t numDispatchedPps;

    //
    // The number of dispatched IMU messages
    std::size_t numDispatchedImu;

    //
    // The number of dispatched compressed images
    std::size_t numDispatchedCompressedImage;

    //
    // The number of dispatched ground surface spline events
    std::size_t numDispatchedGroundSurfaceSpline;

    //
    // The number of dispatched AprilTag detection events
    std::size_t numDispatchedAprilTagDetections;

    //
    // The number of dispatached feature detections
    std::size_t numDispatchedFeatureDetections;
};

} // namespace system
} // namespace multisense
} // namespace crl

#if defined (_MSC_VER)
#pragma warning (pop)
#endif

#endif // LibMultiSense_MultiSenseTypes_hh
