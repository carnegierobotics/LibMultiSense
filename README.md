# LibMultiSense

- [Hello World](#hello-world)
  - [Point Cloud Generation](#point-cloud-generation)
  - [Depth Image Generation](#depth-image-generation)
  - [Camera Configuration](#camera-configuration)
- [Optional Dependencies](#optional-dependencies)
- [Installation](#installation)
- [Documentation](#documentation)
- [Support](#support)

LibMultiSense is a C++ library used to interface with the MultiSense S
family of sensors from Carnegie Robotics. For more information on the
various MultiSense products please visit
https://carnegierobotics.com/products

For more detailed documentation on general MultiSense operation
please visit
https://docs.carnegierobotics.com/

LibMultiSense was recently refactored to have a new API. The following examples in the README all assume the user
is using the new API. To build with the new API, the following CMake arguments should be set.

    -DBUILD_LEGACY_API=0FF

### Hello World

#### Python

```python
import libmultisense as lms

channel_config = lms.ChannelConfig()
channel_config.ip_address = "10.66.171.21"
channel = lms.Channel.create(channel_config)

channel.start_streams([lms.DataSource.LEFT_RECTIFIED_RAW])

while True:
    frame = channel.get_next_image_frame()
    if frame:
        for source, image in frame.images.items():
            cv2.imwrite(str(source) + ".png", image.as_array)
```

#### C++

```c++
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

int main()
{
    const auto channel = lms::Channel::create(lms::Channel::ChannelConfig{"10.66.171.21"});
    channel->start_streams({lms::DataSource::LEFT_RECTIFIED_RAW});

    while(true):
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

### Optional Dependencies

#### OpenCV

LibMultiSense optionally has OpenCV utility functions to make the LibMultiSense client API easier to integrate
with existing systems. To build build the OpenCV helpers the following CMake argument should be set

    -DBUILD_OPENCV=ON

This will require a system installation of OpenCV, or an installation which can be pointed to with CMake's
`CMAKE_PREFIX_PATH` argument

#### nlohmann json

LibMultiSense optionally uses nlohmann_json for serialization of base LibMultiSense types. To build build the 
nlohmann_json serialization helpers the following CMake argument should be set

    -DBUILD_JSON_SERIALIZATION=ON

This will require a system installation of nlohmann_json, or an installation which can be pointed to with CMake's
`CMAKE_PREFIX_PATH` argument

### Installation

#### Linux

##### Python

LibMultiSense uses pybind11 to generate python bindings for the base LibMultiSense API

To install the LibMultiSense python bindings

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > pip install .

##### C++

LibMultiSense uses CMake for its build system.

To build the standalone LibMultiSense library and demonstration applications.

    # Note this only needs to be run once before building
    > sudo apt install build-essential

    > git clone https://github.com/carnegierobotics/LibMultiSense.git
    > cd LibMultiSense
    > mkdir build
    > cd build && cmake -DBUILD_LEGACY_API=OFF -DCMAKE_INSTALL_PREFIX=../install ..
    > make install
    > cd ../install

To build the standalone LibMultiSense library without the demonstration applications,
set the cmake variable `-DMULTISENSE_BUILD_UTILITIES=OFF`

Integrating LibMultiSense into an existing CMake project is easy. There are two
primary methods for integration: a local install on your system, or a submodule
clone within your repository

###### Local Installation

LibMultiSense is installed on your system (i.e. in a location like /opt/multisense)

    find_package(MultiSense)
    target_link_libraries(<your-library-or-binary> MultiSense)

When running CMake, make sure to specify the location of the LibMultiSense install
via `-DCMAKE_PREFIX_PATH`

##### Git Submodule

Clone the LibMultiSense repository into the existing project's source tree.
In the main CMakeLists.txt file of the project, add the following lines:

     include_directories(LibMultiSense/source/LibMultiSense)
     add_subdirectory(LibMultiSense/source/LibMultiSense)

#### Windows

LibMultiSense uses CMake to create a Microsoft Visual Studio project file used
to build the LibMultiSense DLL.

Download and install CMake on Windows (http://www.cmake.org/download/), making
sure CMake is included included in the system PATH.

Clone LibMultiSense to the Windows machine using a Windows Git client.

Open a command prompt and execute the following commands:

    > cd <LibMultiSense_checkout_directory>
    > mkdir build
    > cd build
    > cmake ..

This will create a LibMultiSense.sln Visual Studio Solution file in the build directory.
Open the solution file with Visual Studio (http://msdn.microsoft.com/en-us/vstudio/aa718325.aspx)
and build the Solution.

### Documentation

Documentation of high-level LibMutliSense concepts can be found
[here](https://docs.carnegierobotics.com/docs/software/libmultisense.html)

Doxygen documentation can be built for LibMultisense by running the Doxygen
configuration file located in the docs directory

    > cd LibMultiSense/docs
    > doxygen Doxyfile

HTML and LaTex documentation will be generated in the docs directory.

Usage examples are included in the Doxygen documentation.

### Support

To report an issue with this library or request a new feature,
please use the [GitHub issues system](https://github.com/carnegierobotics/LibMultiSense/issues)

For product support, please see the [support section of our website](https://carnegierobotics.com/support)
Individual support requests can be created in our [support portal](https://carnegierobotics.com/submitaticket)


### Camera Configuration

Camera settings like resolution, exposure, FPS, gain, gamma, and white balance can be configured
via the LibMultiSense `image::Config`

#### configuration.py

```python
import libmultisense as lms

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    channel = lms.Channel.create(channel_config)
    if not channel:
        print("Invalid channel")
        exit(1)

    config = channel.get_configuration()
    config.frames_per_second = 10.0
    config.width = 960
    config.height = 600
    config.disparities = lms.MaxDisparities.D256
    config.image_config.auto_exposure_enabled = True
    config.image_config.gamma = 2.2
    if channel.set_configuration(config) != lms.Status.OK:
        print("Cannot set configuration")
        exit(1)
```

#### configuration.cc

```c++
#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::ChannelConfig{"10.66.171.21"});
    if (!channel)
    {
        std::cerr << "Failed to create channel" << std::endl;;
        return 1;
    }

    auto config = channel->get_configuration();
    config.frames_per_second = 10.0;
    config.width = 960;
    config.height = 600;
    config.disparities = lms::MultiSenseConfiguration::MaxDisparities::D256;
    config.image_config.auto_exposure_enabled = True;
    config.image_config.gamma = 2.2;
    if (const auto status = channel->set_configuration(config); status != lms::Status::OK)
    {
        std::cerr << "Cannot set config" << std::endl;
        return 1;
    }

    return 0;
}
```

### Point Cloud Generation

Disparity images can be converted to 3D point cloud images using the client API.

The following modified version of the [Hello World](#hello-world) example,
converts disparity images to 3D point clouds colorized using the left rectified image.

#### point_cloud.py

```python
import libmultisense as lms

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    channel = lms.Channel.create(channel_config)
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

```

#### point_cloud.cpp

```c++

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::ChannelConfig{"10.66.171.21"});
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

    while(!done)
    {
        if (const auto image_frame = channel->get_next_image_frame(); image_frame)
        {
            if (const auto point_cloud = lms::create_color_pointcloud<uint8_t>(image_frame.value(),
                                                                               max_range,
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

### Depth Image Generation

Disparity images can be converted to depth images using the client API

The following modified version of the [Hello World](#hello-world) example,
converts disparity images to openni depth images and saves them to disk using OpenCV.

#### depth.py

```python
import libmultisense as lms

def main(args):
    channel_config = lms.ChannelConfig()
    channel_config.ip_address = "10.66.171.21"

    channel = lms.Channel.create(channel_config)
    if not channel:
        print("Invalid channel")
        exit(1)

    if channel.start_streams([lms.DataSource.LEFT_DISPARITY_RAW]) != lms.Status.OK:
        print("Unable to start streams")
        exit(1)

    while True:
        frame = channel.get_next_image_frame()
        if frame:
            # MONO16 depth images are quantized to 1 mm per 1 pixel value to match the OpenNI standard
            depth_image = lms.create_depth_image(frame, lms.PixelFormat.MONO16, lms.DataSource.LEFT_DISPARITY_RAW, 65535)
            if depth_image:
                print("Saving depth image for frame id: ", frame.frame_id)
                cv2.imwrite(str(frame.frame_id) + ".png", depth_image.as_array)
```

#### depth.cpp

```c++

#include <MultiSense/MultiSenseChannel.hh>
#include <MultiSense/MultiSenseUtilities.hh>

namespace lms = multisense;

int main(int argc, char** argv)
{
    const auto channel = lms::Channel::create(lms::Channel::ChannelConfig{"10.66.171.21"});
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
