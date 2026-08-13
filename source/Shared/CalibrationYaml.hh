/**
 * @file Shared/CalibrationYaml.hh
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
 **/

#ifndef CALIBRATION_YAML_HH
#define CALIBRATION_YAML_HH

#include <stdint.h>
#include <cstddef>
#include <limits>
#include <iostream>
#include <iomanip>
#include <map>
#include <string>
#include <utility>
#include <vector>

template<typename T>
std::ostream& writeMatrix (std::ostream& stream, std::string const& name, uint32_t rows, uint32_t columns, T const* data)
{
  stream << name << ": !!opencv-matrix\n";
  stream << "   rows: " << rows << "\n";
  stream << "   cols: " << columns << "\n";
  stream << "   dt: d\n";
  stream << "   data: [ ";

	stream.precision (17);
	stream << std::scientific;
	for (uint32_t i = 0; i < rows; i++)    {
    if (i != 0) {
      stream << ",\n";
      stream << "           ";
    }
    for (uint32_t j = 0; j < columns; j++)  {
      if (j != 0)    {
        stream << ", ";
      }
      stream << std::setw(22) << data[i * columns + j];
    }
  }
  stream << " ]\n";
  return stream;
}

template<typename T>
std::ostream& writeCompactMatrix(std::ostream& stream,
                                 std::string const& name,
                                 uint32_t rows,
                                 uint32_t columns,
                                 T const* data)
{
  stream << name << ":\n";
  stream << "  rows: " << rows << "\n";
  stream << "  cols: " << columns << "\n";
  stream << "  data: [";

  const std::streamsize oldPrecision = stream.precision();
  const std::ios_base::fmtflags oldFlags = stream.flags();
  stream.precision(std::numeric_limits<T>::max_digits10);
  stream << std::defaultfloat;
  for (uint32_t i = 0; i < rows * columns; ++i)
  {
    if (i != 0)
    {
      stream << ", ";
    }
    stream << data[i];
  }
  stream << "]\n";
  stream.precision(oldPrecision);
  stream.flags(oldFlags);
  return stream;
}

class Expect
{
private:
    std::string m_value;

public:
    Expect (std::string const& value) :
        m_value (value)
    {
    }

    std::string const& value () const
    {
        return this->m_value;
    }
};

inline std::istream& operator >> (std::istream& stream, Expect const& expect)
{
    stream >> std::ws;

    for (std::string::const_iterator iter = expect.value ().begin (); iter != expect.value ().end (); ++iter)
    {
        if (*iter == ' ')
        {
            stream >> std::ws;
            continue;
        }
        if (stream.get () != *iter)
        {
            stream.clear (std::ios_base::failbit);
            break;
        }
    }

    return stream;
}

template<typename T>
std::istream& operator >> (std::istream& stream, std::vector<T>& data)
{
    while (stream.good())
    {
        stream >> std::ws;
        if (stream.peek() == ']')
        {
            stream.get();
            break;
        }

        if (stream.peek() == '[')
        {
            stream.get();
            stream >> data;
        }
        else
        {
            T value{};
            if (!(stream >> value))
            {
                break;
            }
            data.push_back(value);
        }

        stream >> std::ws;
        const int separator = stream.get();
        if (separator == ']')
        {
            break;
        }
        if (separator != ',')
        {
            stream.setstate(std::ios_base::failbit);
            break;
        }
    }

    return stream;
}

inline std::istream& parseYaml(std::istream& stream,
                              std::map<std::string, std::vector<float>>& data)
{
    while (stream.good())
    {
        stream >> std::ws;
        if (stream.eof())
        {
            break;
        }
        const int next = stream.peek();
        if (next == std::char_traits<char>::eof())
        {
            break;
        }
        if (next == '%' || next == '-' || next == '#')
        {
            std::string comment;
            std::getline(stream, comment);
            continue;
        }

        std::string name;
        stream >> name;
        if (name.empty())
        {
            break;
        }
        if (name.back() != ':')
        {
            stream.setstate(std::ios_base::failbit);
            break;
        }
        name.pop_back();

        std::vector<float> arrayContents;
        stream >> std::ws;
        if (stream.peek() == '[')
        {
            stream.get();
            stream >> arrayContents;
        }
        else
        {
            uint32_t rows = 0;
            uint32_t columns = 0;
            if (stream.peek() == '!')
            {
                stream >> Expect("!!opencv-matrix");
            }
            stream >> Expect("rows:") >> rows;
            stream >> Expect("cols:") >> columns;

            std::string field;
            stream >> field;
            if (field == "dt:")
            {
                std::string dataType;
                stream >> dataType;
                stream >> field;
            }
            if (field != "data:")
            {
                stream.setstate(std::ios_base::failbit);
            }
            else
            {
                stream >> Expect("[") >> arrayContents;
            }

            if (rows != 0 && columns > std::numeric_limits<uint32_t>::max() / rows)
            {
                stream.setstate(std::ios_base::failbit);
            }
            else if (static_cast<size_t>(rows) * columns != arrayContents.size())
            {
                stream.setstate(std::ios_base::failbit);
            }
        }

        if (!stream.fail() && !data.emplace(name, std::move(arrayContents)).second)
        {
            stream.setstate(std::ios_base::failbit);
        }
    }

    return stream;
}

#endif //CALIBRATION_YAML_HH
