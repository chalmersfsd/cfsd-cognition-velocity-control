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

VelocityControl::VelocityControl(float useConstantSpeed, float ayLimit)
  : calculateSpeed{NULL}
  , m_aimPoint{}
  , m_useConstantSpeed{useConstantSpeed}
  , m_ayLimit{ayLimit}

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
  // Point at constant speed or changing speed function based on program input
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
  // --------------- CALL SPEED CALCULATION FUNCTION ------------------
  float speed = (this->*calculateSpeed)();
  

  // ------------ RETURN CORRECT MESSAGE TYPE -------------------
  opendlv::proxy::GroundSpeedRequest speedRequest;
  speedRequest.groundSpeed(speed);

  return speedRequest;
}

float VelocityControl::constantSpeed()
{
  return m_useConstantSpeed;
}

float VelocityControl::dynamicSpeed()
{
  float aimDistance;
  Eigen::MatrixXf path(6,2);
  {
    std::lock_guard<std::mutex> lock(m_readingsMutex);

    aimDistance = m_aimPoint.distance();
    path << 0.936318f, 1.69872f,
            1.36093f, 1.90181f,
            1.78554f, 2.10489f,
            2.21016f, 2.30798f,
            2.63477f, 2.51106f,
            3.05938f, 2.71415f;
  }
  std::cout << path << std::endl;

  // https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
  // Go through all points and triangulate path radius except the two last points
  Eigen::VectorXf curveRadii(path.rows() - 2);
  for (int i = 0; i < path.rows() - 2; i++)
  {
    float A = pointDistance(path, i + 1, i);
    float B = pointDistance(path, i + 2, i + 1);
    float C = pointDistance(path, i + 2, i);

    // sort side lengths as A >= B && B >= C
    if (A < B)
    {
      std::swap(A, B);
    }
    if (A < C)
    {
      std::swap(A, C);
    }
    if (B < C)
    {
      std::swap(B, C);
    }

    // Set large radius to meet velocity limit to avoid inf
    if (C - (A - B) <= 0) {
      curveRadii[i] = 10000.0f;
    } else {
      float area = 0.25f * sqrtf((A + (B + C)) * (C - (A - B)) * (C + (A - B)) * (A + (B - C)));
      curveRadii[i] = A * B * C / (4.0f * area);
      // std::cout << "(" << A << ", " << B << ", " << C << ")" << std::endl;
    }
  }

  // v = sqrt(ay * R)
  Eigen::VectorXf speedPoints = m_ayLimit * curveRadii;
  std::cout << "Before: " << speedPoints << std::endl;
  speedPoints = speedPoints.array().sqrt();

  std::cout << "After: " << speedPoints << std::endl;

  return aimDistance;
}

// ############################ UTILITY FUNCTIONS ###################################

float VelocityControl::pointDistance(Eigen::MatrixXf &points, int i, int j) {
  return std::sqrt(std::pow(points(i,0) - points(j,0), 2.0f) + std::pow(points(i,1) - points(j,1), 2.0f));
}

void VelocityControl::setAimPoint(opendlv::logic::action::AimPoint aimPoint)
{
  std::lock_guard<std::mutex> lock(m_readingsMutex);
  m_aimPoint = aimPoint;
}