# LibMultiSense

LibMultiSense is a C++ and Python library designed to simplify interaction with the MultiSense S family of stereo
sensors developed by Carnegie Robotics. It provides a comprehensive, easy-to-use API for capturing and processing
stereo sensor data an generating depth images, color images, and 3D point clouds.

**Official product page:** [Carnegie Robotics MultiSense Products](https://carnegierobotics.com/products)

**Detailed documentation:** [LibMultiSense Documentation](https://docs.carnegierobotics.com/docs/software/libmultisense.html)

LibMultiSense was recently refactored to have a new API. The following examples in the README all assume the user
is using the new API. To build with the new API, the following CMake arguments should be set.

    -DBUILD_LEGACY_API=0FF

The README for the Legacy API can be found [here](source/Legacy/README.md).

The LibMultiSense C++ and Python library has been tested with the following operating systems

- Ubuntu
    - 20.04
    - 22.04
    - 24.04
 - MacOS Sequoia
 - Windows 11

## Table of Contents

- [Client Networking Prerequisite](#client-networking-prerequisite)
- [Quickstart Guide](#quickstart-guide)
  - [Python](#python)
  - [C++](#c)
- [Optional Dependencies](#optional-dependencies)
  - [OpenCV](#opencv)
  - [nlohmann_json](#nlohmann_json)
  - [pybind11](#pybind11)
  - [googletest](#googletest)
- [Installation](#installation)
  - [Linux](#linux)
    - [Python](#python-1)
    - [C++](#c-1)
  - [MacOS](#macos)
    - [Python](#python-2)
    - [C++](#c-2)
  - [Windows](#windows)
    - [Python](#python-3)
    - [C++](#c-3)
- [CMake Project Integration](#cmake-project-integration)
  - [Local Installation](#local-installation)
  - [Git Submodule](#git-submodule)
- [Documentation](#documentation)
- [Support](#support)
- [Camera Configuration](#camera-configuration)
  - [Python](#python-4)
  - [C++](#c-4)
- [Point Cloud Generation](#point-cloud-generation)
  - [Python](#python-5)
  - [C++](#c-5)
- [Depth Image Generation](#depth-image-generation)
  - [Python](#python-6)
  - [C++](#c-6)
- [Color Image Generation](#color-image-generation)
  - [Python](#python-7)
  - [C++](#c-7)
- [IMU Data Streaming](#imu-data-streaming)
  - [Python](#python-8)
  - [C++](#c-8)
- [Feature Rendering](#feature-rendering)
  - [Python](#python-9)
  - [C++](#c-9)

## Client Networking Prerequisite

The MultiSense comes preconfigured with a static 10.66.171.21 IP address with a /24 subnet. To connect to the
MultiSense, a client machine must be updated with an IP address on the 10.66.171 subnet.

Please see the [host network configuration](https://docs.carnegierobotics.com/network/network.html) for details
on how to set a client machine's IP address and MTU.


##  Quickstart Guide

Below are minimal examples demonstrating basic usage of LibMultiSense to capture rectified images from the camera.

Before running the examples make sure [LibMultiSense is installed](#Installation), and your machines network is
[properly configured](#client-networking-prerequisite).

For new users, it's recommended to start with the Python version of LibMultiSense

### Python

Install the LibMultiSenes python client and OpenCV dependency via

```
pip install libmultisense opencv-python
```

```python
import libmultisense as lms
import cv2

channel_config = lms.ChannelConfig()
channel_config.ip_address = "10.66.171.21"
with lms.Channel.create(channel_config) as channel:
    channel.start_streams([lms.DataSource.LEFT_RECTIFIED_RAW, lms.DataSource.RIGHT_RECTIFIED_RAW])

    while True:
        frame = channel.get_next_image_frame()
        if frame:
            for source, image in frame.images.items():
                cv2.imwrite(str(source) + ".png", image.as_array)
```

### C++

```c++
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

int main()
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    channel->start_streams({lms::DataSource::LEFT_RECTIFIED_RAW, lms::DataSource::RIGHT_RECTIFIED_RAW});

    while(true)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            for (const auto &[source, image]: image_frame->images)
            {
                const auto path = std::to_string(image_frame->frame_id) +  "_" +
                                  std::to_string(static_cast<int>(source)) + ".png";
                lms::write_image(image, path);
            }
        }
    }
    return 0;
}
```

---

## Optional Dependencies When Building From Source

### OpenCV

LibMultiSense optionally has OpenCV utility functions to make the LibMultiSense client API easier to integrate
with existing systems. To build the OpenCV helpers the following CMake argument should be set

    -DBUILD_OPENCV=ON

This will require a system installation of OpenCV, or an installation which can be pointed to with CMake's
`CMAKE_PREFIX_PATH` argument

### nlohmann json

LibMultiSense optionally uses nlohmann_json for serialization of base LibMultiSense types. To build the
nlohmann_json serialization helpers the following CMake argument should be set

    -DBUILD_JSON_SERIALIZATION=ON

This will require a system installation of nlohmann_json, or an installation which can be pointed to with CMake's
`CMAKE_PREFIX_PATH` argument

### pybind11

LibMultiSense optionally uses pybind11 to generate python bindings for the C++ API. To build the pybind11 python
bindings the following CMake argument should be set

    -DBUILD_PYTHON_BINDINGS=ON

This will require a system installation of pybind11, or an installation which can be pointed to with CMake's
`CMAKE_PREFIX_PATH` argument

### googletest

LibMultiSense optionally uses googletest for unit testing the C++ API. To build the googletest unit tests
the following CMake argument should be set

    -DBUILD_TESTS=ON

This will require a system installation of googletest, or an installation which can be pointed to with CMake's
`CMAKE_PREFIX_PATH` argument

---

## Installation

### Linux

#### Python

##### PyPi (Recommended)

The following command installs/updates to the latest version of the LibMultisense Python API:

```
pip install --upgrade libmultisense
```

To avoid conflicts with other Python packages, it's recommended to utilize 
[venv](https://docs.python.org/3/library/venv.html) to isolate dependency installations.

##### From Source

LibMultiSense uses pybind11 to generate Python bindings for the base LibMultiSense API. These bindings can be
installed via pip into a Python virtual environment or a local Python project.

To install the LibMultiSense Python bindings

    > sudo apt install build-essential pybind11-dev nlohmann-json3-dev

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > pip install .

#### C++

LibMultiSense uses CMake for its build system.

To build the standalone LibMultiSense library and demonstration applications.

    # Note this only needs to be run once before building
    > sudo apt install build-essential nlohmann-json3-dev

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > mkdir build
    > cd build && cmake -DBUILD_LEGACY_API=OFF -DBUILD_JSON_SERIALIZATION=ON -DCMAKE_INSTALL_PREFIX=../install ..
    > make install
    > cd ../install

To build the standalone LibMultiSense library without the demonstration applications,
set the cmake variable `-DMULTISENSE_BUILD_UTILITIES=OFF`

---

### MacOS

#### Python

##### PyPi (Recommended)

The following command installs/updates to the latest version of the LibMultisense Python API:

```
pip install --upgrade libmultisense
```

To avoid conflicts with other Python packages, it's recommended to utilize 
[venv](https://docs.python.org/3/library/venv.html) to isolate dependency installations.

##### From Source

LibMultiSense uses pybind11 to generate Python bindings for the base LibMultiSense API. These bindings can be
installed via pip into a Python virtual environment or a local Python project.

To install the LibMultiSense Python bindings

    > brew install pybind11 nlohmann-json

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > pip install .

#### C++

LibMultiSense uses CMake for its build system.

To build the standalone LibMultiSense library and demonstration applications.

    # Note this only needs to be run once before building
    > brew install nlohmann-json

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > mkdir build
    > cd build && cmake -DBUILD_LEGACY_API=OFF -DBUILD_JSON_SERIALIZATION=ON -DCMAKE_INSTALL_PREFIX=../install ..
    > make install
    > cd ../install

To build the standalone LibMultiSense library without the demonstration applications,
set the cmake variable `-DMULTISENSE_BUILD_UTILITIES=OFF`

---

### Windows

#### Python

##### PyPi (Recommended)

The following command installs/updates to the latest version of the LibMultisense Python API.

After installing Python via the Microsoft Store execute the following command in a Powershell terminal

```
pip install --upgrade libmultisense
```

To avoid conflicts with other Python packages, it's recommended to utilize
[venv](https://docs.python.org/3/library/venv.html) to isolate dependency installations.

##### From Source

LibMultiSense uses pybind11 to generate Python bindings for the base LibMultiSense API. These bindings can be
installed via pip into a Python virtual environment or a local Python project. To ensure Windows has the proper build
tools installed, please install Microsoft Visual Studio with the C++ and CMake extensions.

Note you will need to have a version of Python installed on your Windows system. This was tested with
Python 3.9 installed via the Microsoft Store on Windows 11.

To install the LibMultiSense Python bindings open a powershell terminal and execute the following commands

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > git clone https://github.com/microsoft/vcpkg.git
    > ./vcpkg/bootstrap-vcpkg.bat

    > $Env:VCPKG_ROOT = ./vcpkg

    > pip install .

#### C++

LibMultiSense uses CMake and vcpkg to build the LibMultiSense library. To ensure Windows has the proper build
tools installed, please install Microsoft Visual Studio with the C++ and CMake extensions.

Open a powershell terminal and execute the following commands:

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > git clone https://github.com/microsoft/vcpkg.git
    > ./vcpkg/bootstrap-vcpkg.bat

    > $Env:VCPKG_ROOT = ./vcpkg

    > cmake --build build --config Release --target install

---

### CMake Project Integration

Integrating LibMultiSense into an existing CMake project is easy. There are two
primary methods for integration: a local install on your system, or a submodule
clone within your repository

#### Local Installation

LibMultiSense is installed on your system (i.e. in a location like /opt/multisense)

    find_package(MultiSense)
    target_link_libraries(<your-library-or-binary> MultiSense)

When running CMake, make sure to specify the location of the LibMultiSense install
via `-DCMAKE_PREFIX_PATH`

#### Git Submodule

Clone the LibMultiSense repository into the existing project's source tree.
In the main CMakeLists.txt file of the project, add the following lines:

     include_directories(LibMultiSense/source/LibMultiSense)
     add_subdirectory(LibMultiSense/source/LibMultiSense)

---

## Documentation

Documentation of high-level LibMultiSense concepts can be found
[here](https://docs.carnegierobotics.com/docs/software/libmultisense.html)

Doxygen documentation can be built for LibMultisense by running the Doxygen
configuration file located in the docs directory

    > cd LibMultiSense/docs
    > doxygen Doxyfile

HTML and LaTex documentation will be generated in the docs directory.

Usage examples are included in the Doxygen documentation.

---

## Support

To report an issue with this library or request a new feature,
please use the [GitHub issues system](https://github.com/carnegierobotics/LibMultiSense/issues)

For product support, please see the [support section of our website](https://carnegierobotics.com/support)
Individual support requests can be created in our [support portal](https://carnegierobotics.com/submitaticket)

---

## Camera Configuration

Camera settings like resolution, exposure, FPS, gain, gamma, and white balance can be configured
via the LibMultiSense `image::Config`

### Python

```python
import libmultisense as lms
import cv2

def main():
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        config = channel.get_config()
        config.frames_per_second = 10.0
        config.width = 960
        config.height = 600
        config.disparities = lms.MaxDisparities.D256
        config.image_config.auto_exposure_enabled = True
        config.image_config.gamma = 2.2
        if channel.set_config(config) != lms.Status.OK:
            print("Cannot set configuration")
            exit(1)

if __name__ == "__main__":
    main()
```

### C++

```c++
#include <iostream>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    auto config = channel->get_config();
    config.frames_per_second = 10.0;
    config.width = 960;
    config.height = 600;
    config.disparities = lms::MultiSenseConfig::MaxDisparities::D256;
    config.image_config.auto_exposure_enabled = true;
    config.image_config.gamma = 2.2;
    if (const auto status = channel->set_config(config); status != lms::Status::OK)
    {
        std::cerr << "Cannot set config" << std::endl;
        return 1;
    }

    return 0;
}
```
---

## Point Cloud Generation

Disparity images can be converted to 3D point cloud images using the client API.

The following modified version of the [Quickstart](#quickstart-guide) example,
converts disparity images to 3D point clouds colorized using the left rectified image.

### Python

```python
import libmultisense as lms
import cv2

def main():
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        if channel.start_streams([lms.DataSource.LEFT_RECTIFIED_RAW, lms.DataSource.LEFT_DISPARITY_RAW]) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        while True:
            frame = channel.get_next_image_frame()
            if frame:
                point_cloud = lms.create_gray8_pointcloud(frame,
                                                         args.max_range,
                                                         lms.DataSource.LEFT_RECTIFIED_RAW,
                                                         lms.DataSource.LEFT_DISPARITY_RAW)

                print("Saving pointcloud for frame id: ", frame.frame_id)
                lms.write_pointcloud_ply(point_cloud, str(frame.frame_id) + ".ply")

if __name__ == "__main__":
    main()
```

### C++

```c++
#include <iostream>

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

volatile bool done = false;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    if (const auto status = channel->start_streams({lms::DataSource::LEFT_RECTIFIED_RAW,
                                                    lms::DataSource::LEFT_DISPARITY_RAW}); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    const double max_range_m = 20.0;

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            if (const auto point_cloud = lms::create_color_pointcloud<uint8_t>(image_frame.value(),
                                                                               max_range_m,
                                                                               lms::DataSource::LEFT_RECTIFIED_RAW,
                                                                               lms::DataSource::LEFT_DISPARITY_RAW); point_cloud)
            {
                std::cout << "Saving pointcloud for frame id: " << image_frame->frame_id << std::endl;
                lms::write_pointcloud_ply(point_cloud.value(), std::to_string(image_frame->frame_id) + ".ply");
            }
        }
    }

    return 0;
}
```

---

## Depth Image Generation

Disparity images can be converted to depth images using the client API

The following modified version of the [Quickstart](#quickstart-guide) example,
converts disparity images to openni depth images and saves them to disk using OpenCV.

### Python

```python
import libmultisense as lms
import cv2

def main():
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        if channel.start_streams([lms.DataSource.LEFT_DISPARITY_RAW]) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        # Set to true to save the depth image in the frame of the aux color camera
        in_aux_frame = False

        while True:
            frame = channel.get_next_image_frame()
            if frame:
                # MONO16 depth images are quantized to 1 mm per 1 pixel value to match the OpenNI standard.
                # For example, a depth image pixel with a value of 10 would correspond to a depth of 10mm
                depth_image = lms.create_depth_image(frame, lms.PixelFormat.MONO16, lms.DataSource.LEFT_DISPARITY_RAW, in_aux_frame, 65535)
                if depth_image:
                    print("Saving depth image for frame id: ", frame.frame_id)
                    cv2.imwrite(str(frame.frame_id) + ".png", depth_image.as_array)

if __name__ == "__main__":
    main()
```

### C++

```c++
#include <iostream>

#include <opencv2/opencv.hpp>

#define HAVE_OPENCV 1
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

volatile bool done = false;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    if (const auto status = channel->start_streams({lms::DataSource::LEFT_DISPARITY_RAW}); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    // Set to true to save the depth image in the frame of the aux color camera
    const bool in_aux_frame = false;

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            //
            // MONO16 depth will be quantized to mm to match OpenNI's depth format
            //
            if (const auto depth_image = lms::create_depth_image(image_frame.value(),
                                                                 lms::Image::PixelFormat::MONO16,
                                                                 lms::DataSource::LEFT_DISPARITY_RAW,
                                                                 in_aux_frame,
                                                                 65535); depth_image)
            {
                std::cout << "Saving depth image for frame id: " << image_frame->frame_id << std::endl;
                cv::imwrite(std::to_string(image_frame->frame_id) + ".png", depth_image->cv_mat());
            }
        }
    }

    return 0;
}
```

---

## Color Image Generation

Luma and Chroma Aux images can be converted to BGR color images using the client API

The following modified version of the [Quickstart](#quickstart-guide) example,
converts luma and chroma aux images to BGR images and saves them to disk using OpenCV.

### Python

```python
import libmultisense as lms
import cv2

def main():
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        if channel.start_streams([lms.DataSource.AUX_RAW]) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        while True:
            frame = channel.get_next_image_frame()
            if frame:
                bgr = lms.create_bgr_image(frame, lms.DataSource.AUX_RAW)
                if bgr:
                    cv2.imwrite(str(frame.frame_id) + ".png", bgr.as_array)

if __name__ == "__main__":
    main()
```

### C++

```c++
#include <iostream>

#include <opencv2/opencv.hpp>

#define HAVE_OPENCV 1
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

volatile bool done = false;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    if (const auto status = channel->start_streams({lms::DataSource::AUX_RAW}); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            if (const auto bgr = create_bgr_image(image_frame.value(), lms::DataSource::AUX_RAW); bgr)
            {
                cv::imwrite(std::to_string(image_frame->frame_id) + ".png", bgr->cv_mat());
            }
        }
    }

    return 0;
}
```

---

=======
## IMU Data Streaming

LibMultiSense supports streaming IMU data from the camera. The IMU must first be configured
to enable the desired sensors (accelerometer, gyroscope) and set their sample rates
and ranges.

### Python

```python

import libmultisense as lms

def main():
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        #
        # Get the current configuration
        #
        config = channel.get_config()

        #
        # Configure the IMU. We first need to get the IMU info to find supported rates and ranges
        #
        info = channel.get_info()
        if not info.imu:
            print("Sensor does not have an IMU")
            exit(1)

        imu_config = lms.ImuConfig()
        imu_config.samples_per_frame = 10 # Number of samples per ImuFrame

        # Enable Accelerometer. Select appropriate rate/range
        if info.imu.accelerometer:
            accel_mode = lms.ImuOperatingMode()
            accel_mode.enabled = True
            accel_mode.rate = info.imu.accelerometer.rates[0]
            accel_mode.range = info.imu.accelerometer.ranges[0]
            imu_config.accelerometer = accel_mode

        # Enable Gyroscope. Select appropriate rate/range
        if info.imu.gyroscope:
            gyro_mode = lms.ImuOperatingMode()
            gyro_mode.enabled = True
            gyro_mode.rate = info.imu.gyroscope.rates[0]
            gyro_mode.range = info.imu.gyroscope.ranges[0]
            imu_config.gyroscope = gyro_mode

        config.imu_config = imu_config
        if channel.set_config(config) != lms.Status.OK:
            print("Failed to set IMU configuration")
            exit(1)

        #
        # Start the IMU stream
        #
        if channel.start_streams([lms.DataSource.IMU]) != lms.Status.OK:
            print("Unable to start IMU stream")
            exit(1)

        while True:
            imu_frame = channel.get_next_imu_frame()
            if imu_frame:
                for sample in imu_frame.samples:
                    if sample.accelerometer:
                        print(f"Accel: x={sample.accelerometer.x}, y={sample.accelerometer.y}, z={sample.accelerometer.z}")
                    if sample.gyroscope:
                        print(f"Gyro: x={sample.gyroscope.x}, y={sample.gyroscope.y}, z={sample.gyroscope.z}")

if __name__ == "__main__":
    main()
```

### C++

```c++
#include <iostream>
#include <MultiSense/MultiSenseChannel.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;
        return 1;
    }

    //
    // Get the current configuration
    //
    auto config = channel->get_config();

    //
    // Configure the IMU. We first need to get the IMU info to find supported rates and ranges
    //
    const auto info = channel->get_info();
    if (!info.imu)
    {
        std::cerr << "Sensor does not have an IMU" << std::endl;
        return 1;
    }

    lms::MultiSenseConfig::ImuConfig imu_config;
    imu_config.samples_per_frame = 10;

    // Enable Accelerometer
    if (info.imu->accelerometer)
    {
        lms::MultiSenseConfig::ImuConfig::OperatingMode accel_mode;
        accel_mode.enabled = true;
        accel_mode.rate = info.imu->accelerometer->rates[0];
        accel_mode.range = info.imu->accelerometer->ranges[0];
        imu_config.accelerometer = accel_mode;
    }

    // Enable Gyroscope
    if (info.imu->gyroscope)
    {
        lms::MultiSenseConfig::ImuConfig::OperatingMode gyro_mode;
        gyro_mode.enabled = true;
        gyro_mode.rate = info.imu->gyroscope->rates[0];
        gyro_mode.range = info.imu->gyroscope->ranges[0];
        imu_config.gyroscope = gyro_mode;
    }

    config.imu_config = imu_config;
    if (const auto status = channel->set_config(config); status != lms::Status::OK)
    {
        std::cerr << "Failed to set IMU configuration: " << lms::to_string(status) << std::endl;
        return 1;
    }

    //
    // Start the IMU stream
    //
    if (const auto status = channel->start_streams({lms::DataSource::IMU}); status != lms::Status::OK)
    {
        std::cerr << "Cannot start IMU stream: " << lms::to_string(status) << std::endl;
        return 1;
    }

    while(true)
    {
        if (const auto imu_frame = channel->get_next_imu_frame(); imu_frame)
        {
            for (const auto& sample : imu_frame->samples)
            {
                if (sample.accelerometer)
                {
                    std::cout << "Accel: x=" << sample.accelerometer->x
                              << ", y=" << sample.accelerometer->y
                              << ", z=" << sample.accelerometer->z << std::endl;
                }
                if (sample.gyroscope)
                {
                    std::cout << "Gyro: x=" << sample.gyroscope->x
                              << ", y=" << sample.gyroscope->y
                              << ", z=" << sample.gyroscope->z << std::endl;
                }
            }
        }
    }

    return 0;
}
```

---

## Feature Rendering

LibMultiSense supports retrieving image features computed on-camera. These are synchronized with the
corresponding image frames.

The following example demonstrates how to retrieve and render features on a rectified image.

### Python

```python
import libmultisense as lms
import cv2

def main():
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    with lms.Channel.create(channel_config) as channel:
        if not channel:
            print("Invalid channel")
            exit(1)

        # Set the feature detector config to enable the feature detector
        config = channel.get_config()
        config.feature_detector_config = lms.FeatureDetectorConfig()
        config.feature_detector_config.number_of_features = 1500
        config.feature_detector_config.grouping_enabled = True
        channel.set_config(config)

        # Start both the rectified image and the corresponding feature stream
        sources = [lms.DataSource.LEFT_MONO_RAW, lms.DataSource.LEFT_ORB_FEATURES]
        if channel.start_streams(sources) != lms.Status.OK:
            print("Unable to start streams")
            exit(1)

        while True:
            frame = channel.get_next_image_frame()
            if frame and frame.has_image(lms.DataSource.LEFT_MONO_RAW):
                img = frame.get_image(lms.DataSource.LEFT_MONO_RAW).as_array

                # Convert grayscale to BGR for color rendering
                display_img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

                if frame.has_feature(lms.DataSource.LEFT_ORB_FEATURES):
                    features = frame.get_feature(lms.DataSource.LEFT_ORB_FEATURES)
                    print(f"Frame {frame.frame_id}: Received {len(features.keypoints)} features")

                    for kp in features.keypoints:
                        cv2.circle(display_img, (int(kp.x), int(kp.y)), 3, (0, 255, 0), -1)

                cv2.imshow("MultiSense Features", display_img)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

if __name__ == "__main__":
    main()
```

### C++

```c++
#include <iostream>
#include <opencv2/opencv.hpp>

#define HAVE_OPENCV 1
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::Config{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;
        return 1;
    }

    //
    // Set the feature detector config to enable the feature detector
    //
    auto config = channel->get_config();
    config.feature_detector_config = lms::MultiSenseConfig::FeatureDetectorConfig{number_of_features, true, 1};
    channel->set_config(config);

    // Start both the rectified image and the corresponding feature stream
    const std::vector<lms::DataSource> sources = {
        lms::DataSource::LEFT_MONO_RAW,
        lms::DataSource::LEFT_ORB_FEATURES
    };

    if (const auto status = channel->start_streams(sources); status != lms::Status::OK)
    {
        std::cerr << "Cannot start streams: " << lms::to_string(status) << std::endl;
        return 1;
    }

    while(true)
    {
        if (const auto frame = channel->get_next_image_frame(); frame)
        {
            if (frame->has_image(lms::DataSource::LEFT_MONO_RAW))
            {
                cv::Mat img = frame->get_image(lms::DataSource::LEFT_MONO_RAW).cv_mat();
                cv::Mat display_img;
                cv::cvtColor(img, display_img, cv::COLOR_GRAY2BGR);

                if (frame->has_feature(lms::DataSource::LEFT_ORB_FEATURES))
                {
                    const auto& features = frame->get_feature(lms::DataSource::LEFT_ORB_FEATURES);

                    // Use the native OpenCV utility to convert keypoints and draw them
                    cv::drawKeypoints(display_img, features.cv_keypoints(), display_img, cv::Scalar(0, 255, 0));
                }

                cv::imshow("MultiSense Features", display_img);
                if (cv::waitKey(1) == 'q') break;
            }
        }
    }

    return 0;
}
```
