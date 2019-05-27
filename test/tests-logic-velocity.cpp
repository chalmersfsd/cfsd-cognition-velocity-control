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

#define CATCH_CONFIG_MAIN  // This tells Catch to provide a main() - only do this in one cpp file
#include "catch.hpp"

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

#include "logic-velocity.hpp"
#include <Eigen/Core>

TEST_CASE("Empty path should not return inf or nan") {
  Eigen::MatrixXf path;

  bool useConstantSpeed = false;
  float ayLimit = 5.0f;
  float velocityLimit = 10.0f;
  float decelerationLimit = 5.0f;
  VelocityControl velocityControl(useConstantSpeed, ayLimit, velocityLimit, decelerationLimit);
  velocityControl.setPath(path);

  opendlv::proxy::GroundSpeedRequest gsr = velocityControl.step();
  float speed = gsr.groundSpeed();

  REQUIRE(std::isfinite(speed));
}


TEST_CASE("Straight line should return velocity limit") {
  Eigen::MatrixXf path(4,2);

  path << 1.0f, 2.0f,
          2.0f, 3.0f,
          3.0f, 4.0f,
          4.0f, 5.0f;

  bool useConstantSpeed = false;
  float ayLimit = 5.0f;
  float velocityLimit = 10.0f;
  float decelerationLimit = 5.0f;
  VelocityControl velocityControl(useConstantSpeed, ayLimit, velocityLimit, decelerationLimit);
  velocityControl.setPath(path);

  opendlv::proxy::GroundSpeedRequest gsr = velocityControl.step();
  float speed = gsr.groundSpeed();

  REQUIRE(speed == Approx(velocityLimit).epsilon(0.01f));
}

TEST_CASE("If fewer than 4 points path should return initial speed request value") {
  Eigen::MatrixXf path(3,2);

  path << 1.0f, 2.0f,
          2.0f, 3.0f,
          3.0f, 4.0f;

  bool useConstantSpeed = false;
  float ayLimit = 5.0f;
  float velocityLimit = 10.0f;
  float decelerationLimit = 5.0f;
  VelocityControl velocityControl(useConstantSpeed, ayLimit, velocityLimit, decelerationLimit);
  velocityControl.setPath(path);

  opendlv::proxy::GroundSpeedRequest gsr = velocityControl.step();
  float speed = gsr.groundSpeed();

  REQUIRE(speed == Approx(0.0f).epsilon(0.01f));
}

TEST_CASE("Return velocityLimit if constant speed is used") {
  Eigen::MatrixXf path(3,2);

  path << 1.0f, -1.0f,
          2.0f, -4.0f,
          3.0f, -9.0f;

  bool useConstantSpeed = true;
  float ayLimit = 5.0f;
  float velocityLimit = 10.0f;
  float decelerationLimit = 5.0f;
  VelocityControl velocityControl(useConstantSpeed, ayLimit, velocityLimit, decelerationLimit);
  velocityControl.setPath(path);

  opendlv::proxy::GroundSpeedRequest gsr = velocityControl.step();
  float speed = gsr.groundSpeed();

  REQUIRE(speed == Approx(velocityLimit).epsilon(0.01f));
}