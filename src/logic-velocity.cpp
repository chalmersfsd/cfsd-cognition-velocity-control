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

#include "cluon-complete.hpp"
#include "logic-velocity.hpp"

VelocityControl::VelocityControl(cluon::OD4Session &od4)
  : m_od4{od4}
  , m_leftWheelSpeed{0.0f}
  , m_rightWheelSpeed{0.0f}

  , m_readingsMutex{}
{
  setUp();
}

VelocityControl::~VelocityControl()
{
  VelocityControl::tearDown();
}

void VelocityControl::setUp()
{
}

void VelocityControl::tearDown()
{
}

void VelocityControl::step()
{
  float rightWheelSpeed;
  float leftWheelSpeed;
  {
    std::lock_guard<std::mutex> lock(m_readingsMutex);

    rightWheelSpeed = m_rightWheelSpeed;
    leftWheelSpeed = m_leftWheelSpeed;
  }

  cluon::data::TimeStamp sampleTime = cluon::time::now();

  opendlv::proxy::TorqueRequest msgTorque;
  msgTorque.torque(rightWheelSpeed);
  m_od4.send(msgTorque, sampleTime, 1500); // Left
  msgTorque.torque(leftWheelSpeed);
  m_od4.send(msgTorque, sampleTime, 1501); // Right
}

void VelocityControl::setLeftWheelSpeed(float speed)
{
  m_leftWheelSpeed = speed;
}

void VelocityControl::setRightWheelSpeed(float speed)
{
  std::lock_guard<std::mutex> lock(m_readingsMutex);
  m_rightWheelSpeed = speed;
}
