/*
 * Copyright (C) 2018  Love Mowitz
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VELOCITYCONTROL_H
#define VELOCITYCONTROL_H

#include <mutex>
#include <cmath>

#include <Eigen/Core>
// Good reference: https://eigen.tuxfamily.org/dox/AsciiQuickReference.txt

#include "opendlv-standard-message-set.hpp"

class VelocityControl {
  public:
    VelocityControl(bool useConstantSpeed, float ayLimit, float velocityLimit, float decelerationLimit);
    ~VelocityControl();

  public:
    opendlv::proxy::GroundSpeedRequest step();

    void setPath(Eigen::MatrixXf path);
    void setAimPoint(opendlv::logic::action::AimPoint aimPoint);

  private:
    void setUp();
    void tearDown();
    float constantSpeed();
    float dynamicSpeed();
    float pointDistance(Eigen::MatrixXf &points, int index1, int index2);

    // Some class method pointer magic
    typedef float (VelocityControl::*vcptr)();
    vcptr calculateSpeed;


  public:
    opendlv::logic::action::AimPoint m_aimPoint;
    Eigen::MatrixXf m_path;
    float m_speedRequest;

    bool m_useConstantSpeed;
    float m_ayLimit;
    float m_velocityLimit;
    float m_decelerationLimit;

    std::mutex m_pathMutex;
    std::mutex m_aimPointMutex;
};
#endif

