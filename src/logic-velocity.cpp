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

VelocityControl::VelocityControl(bool useConstantSpeed, float ayLimit, float velocityLimit, float decelerationLimit)
  : calculateSpeed{NULL}
  , m_aimPoint{}
  , m_path{}
  , m_speedProfile{}
  , m_speedRequest{0.0f}
  , m_useConstantSpeed{useConstantSpeed}
  , m_ayLimit{ayLimit}
  , m_velocityLimit{velocityLimit}
  , m_decelerationLimit{decelerationLimit}

  , m_pathMutex{}
  , m_aimPointMutex{}
  , m_speedProfileMutex{}
{
  setUp();
}

VelocityControl::~VelocityControl()
{
  VelocityControl::tearDown();
}

void VelocityControl::setUp()
{
  std::cout << "Setting up velocity planner..." << std::endl;
  // Point at constant speed or changing speed function based on program input
  if (m_useConstantSpeed) {
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
  // -------------------- CALL SPEED CALCULATION FUNCTION ------------------
  float speed = (this->*calculateSpeed)();
  

  // -------------------- RETURN CORRECT MESSAGE TYPE ----------------------
  opendlv::proxy::GroundSpeedRequest speedRequest;
  speedRequest.groundSpeed(speed);

  return speedRequest;
}

float VelocityControl::constantSpeed()
{
  return m_velocityLimit;
}

float VelocityControl::dynamicSpeed()
{
  float aimAngle;
  Eigen::MatrixXf path;
  {
    std::lock_guard<std::mutex> lock1(m_pathMutex);
    std::lock_guard<std::mutex> lock2(m_aimPointMutex);

    path = m_path;
    aimAngle = m_aimPoint.azimuthAngle();
  }

  // Return previus request if the current path contains too few points
  if (path.rows() < 4) {
    return m_speedRequest;
  }

  // Go through all points and triangulate path radius except the two last points
  Eigen::VectorXf curveRadii(path.rows() - 2);
  for (int i = 0; i < path.rows() - 2; i++) {
    float A = pointDistance(path, i + 1, i);
    float B = pointDistance(path, i + 2, i + 1);
    float C = pointDistance(path, i + 2, i);

    // To avoid inf values see: https://people.eecs.berkeley.edu/~wkahan/Triangle.pdf
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
      float area = 0.25f * std::sqrt((A + (B + C)) * (C - (A - B)) * (C + (A - B)) * (A + (B - C)));
      curveRadii[i] = A * B * C / (4.0f * area);
    }
  }

  // v = sqrt(ay * R)
  Eigen::VectorXf speedProfile(curveRadii.rows());
  for (int i = 0; i < speedProfile.rows(); i++) {
    speedProfile[i] = std::min(std::sqrt(m_ayLimit * curveRadii[i]), m_velocityLimit);
  }
  
  // Back-propagate to ensure reasonable acceleration and deceleration requests
  for (int i = speedProfile.rows()-1; i >= 1; i--) {
    float distance = pointDistance(path, i, i-1);
    float timeBetweenPoints = distance / (speedProfile[i] + speedProfile[i-1]) * 0.5f;
    float requiredAcceleration = (speedProfile[i] - speedProfile[i-1]) / timeBetweenPoints;

    // If the required deceleration is more than the limit, change the speed profile
    if (requiredAcceleration < m_decelerationLimit) {
      speedProfile[i-1] = std::sqrt(std::abs(std::pow(speedProfile[i], 2.0f) - 2.0f * m_decelerationLimit * distance));
    }
  }

  // Store speed profile for viewer
  setSpeedProfile(speedProfile);

  // Return second point in path
  m_speedRequest = speedProfile[1] * (1 + std::abs(aimAngle));
  return m_speedRequest;
}

// ############################ UTILITY FUNCTIONS ###################################

float VelocityControl::pointDistance(Eigen::MatrixXf &points, int i, int j)
{
  // Points is expected to be a nx2 matrix with xy coordinates
  return std::sqrt(std::pow(points(i,0) - points(j,0), 2.0f) + std::pow(points(i,1) - points(j,1), 2.0f));
}


// ################################# SETTERS ########################################

void VelocityControl::setPath(Eigen::MatrixXf path)
{
  std::lock_guard<std::mutex> lock(m_pathMutex);
  m_path = path;
}

void VelocityControl::setAimPoint(opendlv::logic::action::AimPoint aimPoint)
{
  std::lock_guard<std::mutex> lock(m_aimPointMutex);
  m_aimPoint = aimPoint;
}

void VelocityControl::setSpeedProfile(Eigen::MatrixXf speedProfile)
{
  std::lock_guard<std::mutex> lock(m_speedProfileMutex);
  m_speedProfile = speedProfile;
}