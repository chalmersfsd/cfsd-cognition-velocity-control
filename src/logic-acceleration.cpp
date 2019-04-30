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
#include "logic-acceleration.hpp"

Acceleration::Acceleration(cluon::OD4Session &od4)
  : m_od4{od4}
  , m_asState{asState::AS_OFF}
{
  setUp();
}

Acceleration::~Acceleration()
{
  Acceleration::tearDown();
}

void Acceleration::setUp()
{
}

void Acceleration::tearDown()
{
}

void Acceleration::run()
{
  cluon::data::TimeStamp sampleTime = cluon::time::now();

  opendlv::proxy::TorqueRequest msgTorque;
  msgTorque.torque(10.0f);
  m_od4.send(msgTorque, sampleTime, 1000);
}

void Acceleration::setAsState(asState state)
{
  m_asState = state;
}
