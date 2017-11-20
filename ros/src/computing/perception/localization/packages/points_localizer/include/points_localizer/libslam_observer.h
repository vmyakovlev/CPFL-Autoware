/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

//NOTE: this code is NOT observer pattern


#ifndef LIBSLAM_OBSERVER_H
#define LIBSLAM_OBSERVER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <vector>

#include "libdata_structs.h"

struct EvaluationValue
{
    Pose pose;
    double align_time;
    double transformation_probability;
};

class LibSlamObserver
{
    public:
        LibSlamObserver();
        ~LibSlamObserver() = default;
        void update(const EvaluationValue& ev);
        double getTransformationProbabilityScore() const;

    private:
        void readLogFile(const std::string& file_name);
        void writeLogFile(const EvaluationValue& ev) const;
        EvaluationValue searchNeighborPoint(const EvaluationValue& ev) const;
        std::vector<EvaluationValue> ev_reference_array_;

        double transformation_probability_score_;
};

LibSlamObserver::LibSlamObserver()
    :transformation_probability_score_(0)
{
    readLogFile("slam_observer_toyota.csv");
}

template<typename T>
T hypot3d(T x, T y, T z)
{
    return std::sqrt(x*x+y*y+z*z);
}

double LibSlamObserver::getTransformationProbabilityScore() const
{
    return transformation_probability_score_;
}

void LibSlamObserver::update(const EvaluationValue& ev)
{
    if(ev.transformation_probability == 0)
        std::cout << "[WARN]Lost self-position!" << std::endl;

    const auto ev_reference_neighbor = searchNeighborPoint(ev);
    const auto distance_neighbor = hypot3d(ev_reference_neighbor.pose.x - ev.pose.x,
                                           ev_reference_neighbor.pose.y - ev.pose.y,
                                           ev_reference_neighbor.pose.z - ev.pose.z);

    if(distance_neighbor > 10.0)
        std::cout << "[WARN]Far from reference point" << std::endl;

    transformation_probability_score_ = ev.transformation_probability / ev_reference_neighbor.transformation_probability;
    if(transformation_probability_score_ < 0.5)
        std::cout << "[WARN]transformation_probability" << std::endl;

    if(ev.align_time / ev_reference_neighbor.align_time < 0.5)
        std::cout << "[WARN]align_time" << std::endl;
    std::cout << distance_neighbor << " "
              << transformation_probability_score_ << " "
              << ev.align_time / ev_reference_neighbor.align_time << std::endl;
    writeLogFile(ev);
}

EvaluationValue LibSlamObserver::searchNeighborPoint(const EvaluationValue& ev) const
{
    auto ev_neighbor = *std::begin(ev_reference_array_);
    auto distance_neighbor = hypot3d(ev_neighbor.pose.x - ev.pose.x,
                                     ev_neighbor.pose.y - ev.pose.y,
                                     ev_neighbor.pose.z - ev.pose.z);

    for(auto const& ev_reference : ev_reference_array_)
    {
        const auto distance = hypot3d(ev_reference.pose.x - ev.pose.x,
                                      ev_reference.pose.y - ev.pose.y,
                                      ev_reference.pose.z - ev.pose.z);
        if(distance < distance_neighbor)
        {
            ev_neighbor = ev_reference;
            distance_neighbor = distance;
        }
    }
    return ev_neighbor;
}

void LibSlamObserver::readLogFile(const std::string& file_name)
{
    ev_reference_array_.clear();

    std::ifstream file_stream(file_name);
    std::string reading_line;
    while(std::getline(file_stream, reading_line))
    {
        std::stringstream line_ss(reading_line);
        EvaluationValue ev;
        char split;
        line_ss >> ev.pose.x >> split
                >> ev.pose.y >> split
                >> ev.pose.z >> split
                >> ev.pose.roll >> split
                >> ev.pose.pitch >> split
                >> ev.pose.yaw >> split
                >> ev.align_time >> split
                >> ev.transformation_probability;
        std::cout << std::fixed;
        std::cout << ev.transformation_probability << std::endl;
        ev_reference_array_.push_back(ev);
    }
}

void LibSlamObserver::writeLogFile(const EvaluationValue& ev) const
{
    static std::ofstream file_stream("slam_observer.csv");
    file_stream << ev.pose.x << ","
                << ev.pose.y << ","
                << ev.pose.z << ","
                << ev.pose.roll << ","
                << ev.pose.pitch << ","
                << ev.pose.yaw << ","
                << ev.align_time << ","
                << ev.transformation_probability <<std::endl;
}

#endif
