/**
 * @file shared/CalibrationYaml.hh
 *
 * Copyright 2013
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
#include <iostream>
#include <string>
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
	for (uint32_t i = 0; i < rows; i++)
    {
        if (i != 0)
        {
            stream << ",\n";
            stream << "       ";
        }
        for (uint32_t j = 0; j < columns; j++)
        {
            if (j != 0)
            {
                stream << ", ";
            }
            stream << data[i * columns + j];
        }
    }
    stream << " ]\n";
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

std::istream& operator >> (std::istream& stream, Expect const& expect)
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
    char input;
    while (stream.good ())
    {
        input = 0;
        stream >> input;
        if (input == '[')
        {
            stream >> data;
        }
        else
        {
            stream.putback (input);

            T value;
            stream >> value;
            data.push_back (value);
        }

        input = 0;
        stream >> input;
        if (input == ']')
        {
            break;
        }
        else if (input != ',')
        {
            stream.clear (std::ios_base::failbit);
            break;
        }
    }

    return stream;
}

std::istream& parseYaml (std::istream& stream, std::map<std::string, std::vector<float> >& data)
{
    char input;
    while (stream.good ())
    {
        input = 0;
        stream >> input;
        if (input == '%')
        {
            std::string comment;
            std::getline (stream, comment);
            continue;
        }
        stream.putback (input);

        std::string name;
        stream >> name;
        if (name.empty ())
        {
            break;
        }
        if (name[name.size () - 1] != ':')
        {
            stream.clear (std::ios_base::failbit);
            break;
        }
        name.resize (name.size () - 1);

        std::vector<float> arrayContents;
        arrayContents.clear ();

        input = 0;
        stream >> input;
        if (input == '[')
        {
            stream >> arrayContents;
        }
        else
        {
            stream.putback (input);

            uint32_t rows = 0;
            uint32_t columns = 0;
            stream >> Expect ("!!opencv-matrix");
            stream >> Expect ("rows:") >> rows;
            stream >> Expect ("cols:") >> columns;
            stream >> Expect ("dt: d");
            stream >> Expect ("data: [") >> arrayContents;
        }

		if (stream.good())
		{
			data.insert (std::make_pair(name, arrayContents));
		}
		else
		{
			fprintf (stderr, "Error parsing data near array \"%s\"", name.c_str());
		}
    }

    return stream;
}

#endif //CALIBRATION_YAML_HH

