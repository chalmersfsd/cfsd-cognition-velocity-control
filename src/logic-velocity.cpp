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

VelocityControl::VelocityControl(float useConstantSpeed)
  : calculateSpeed{NULL}
  , m_aimPoint{}
  , m_useConstantSpeed{useConstantSpeed}

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
  if (m_useConstantSpeed > 0.1f) {
    calculateSpeed = &VelocityControl::constantSpeed;
  } else {
    calculateSpeed = &VelocityControl::dynamicSpeed;
  }
}

void VelocityControl::tearDown()
{
}

opendlv::proxy::GroundSpeedRequest VelocityControl::step()
{
  float aimDistance;
  {
    std::lock_guard<std::mutex> lock(m_readingsMutex);

    aimDistance = m_aimPoint.distance();
  }


  float speed = (this->*calculateSpeed)(aimDistance);

  opendlv::proxy::GroundSpeedRequest speedRequest;
  speedRequest.groundSpeed(speed);

  return speedRequest;
}

float VelocityControl::constantSpeed(float distance)
{
  // void input arguments since they are not used
  (void) distance;
  return m_constantSpeed;
}

float VelocityControl::dynamicSpeed(float distance)
{
  (void) distance;
  return 5.0f;
}


void VelocityControl::setAimPoint(opendlv::logic::action::AimPoint aimPoint)
{
  std::lock_guard<std::mutex> lock(m_readingsMutex);
  m_aimPoint = aimPoint;
}