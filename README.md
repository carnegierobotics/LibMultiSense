LibMultiSense
Copyright 2014
Carnegie Robotics, LLC
4501 Hatfield Street, Pittsburgh, PA 15201
http://www.carnegierobotics.com

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Carnegie Robotics, LLC nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL CARNEGIE ROBOTICS, LLC BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# LibMultiSense

LibMultiSense is a C++ library used to interface with the MultiSense S
family of sensors from Carnegie Robotics. For more information on the
various MultiSense products please visit
http://carnegierobotics.com/products/

### Installation

LibMultiSense uses CMake for its build system.

To build the standalone LibMultiSense library and demonstration applications.

    > hg clone http://bitbucket.org/crl/LibMultiSense
    > cd LibMultiSense
    > mkdir build
    > cd build && cmake ..
    > make

Integrating LibMultiSense into an existing CMake project is easy. Simply 
clone the LibMultiSense repository into the existing project's source tree.
 In the main CMakeLists.txt file of the project, add the following lines:

  include_directories(LibMultiSense/source/LibMultiSense)
  add_subdirectory(LibMultiSense/source/LibMultiSense)


###Documentation and Examples

LibMultiSense builds as a single shared library which can be linked into
any existing project.

The two header files MultisenseChannel.hh and MultiSenseTypes.hh contain
all the declarations necessary to interface with a MultiSense sensor.

Doxygen documentation can be built for LibMultisense by runnning the Doxygen
configuration file located in the docs directory

    > cd LibMultiSense/docs
    > doxygen Doxyfile

Html and LaTex documentation will be generated in the docs directory.

Usage examples are included in the Doxygen documentation.

###Support

Please direct all issues, questions, and feature requests to
support@carnegierobotics.com




