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
#include "opendlv-standard-message-set.hpp"
#include "cfsd-extended-message-set.hpp"

#include "logic-velocity.hpp"
#include <iostream>

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("cid") || 0 == commandlineArguments.count("ayLimit")
      || 0 == commandlineArguments.count("velocityLimit")
      || 0 == commandlineArguments.count("decelerationLimit")) {
    std::cerr << argv[0] << "Generates the speed requests for Lynx" << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --ayLimit=<Max lateral acceleration> "
      << " --velocityLimit=<Velocity limit> --decelerationLimit=<Deceleration Limit> "
      << "[--constantSpeed=<Constant speed request>] [--verbose=<Verbose or not>]"
      << std::endl;
    std::cerr << "Example: " << argv[0] << "--cid=111 --ayLimit=10.0 --velocityLimit=10.0 --decelerationLimit=5.0 [--constantSpeed] [--verbose]" << std::endl;
    retCode = 1;
  } else {

    // Interface to a running OpenDaVINCI session.  
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    // If useConstantSpeed is defined, velocityLimit is used as constant speed
    bool useConstantSpeed{static_cast<bool>(commandlineArguments.count("constantSpeed"))};

    float ayLimit{static_cast<float>(std::stof(commandlineArguments["ayLimit"]))};
    float velocityLimit{static_cast<float>(std::stof(commandlineArguments["velocityLimit"]))};
    float decelerationLimit{static_cast<float>(std::stof(commandlineArguments["decelerationLimit"]))};
    bool VERBOSE{static_cast<bool>(commandlineArguments.count("verbose"))};

    // VelocityControl object plans the speed
    VelocityControl velocityControl(useConstantSpeed, ayLimit, velocityLimit, decelerationLimit);

    auto onAimPoint{[&velocityControl, &od4, VERBOSE](cluon::data::Envelope &&envelope)
      {
        uint16_t senderStamp = envelope.senderStamp();
        if (senderStamp == 2701) {
          auto aimPoint = cluon::extractMessage<opendlv::logic::action::AimPoint>(std::move(envelope));
          velocityControl.setAimPoint(aimPoint);

          if (VERBOSE) {
            std::cout << "[COGNITION-VELOCITY] Aim point distance: " << aimPoint.distance() << "\n"
              << "[COGNITION-VELOCITY] Aim point azimuth angle: " << aimPoint.azimuthAngle() << std::endl;
          }
        }
      }};

    auto onLocalPath{[&velocityControl, &od4, VERBOSE](cluon::data::Envelope &&envelope)
      {
        uint16_t senderStamp = envelope.senderStamp();
        if (senderStamp == 2601) {
          auto msg = cluon::extractMessage<opendlv::logic::action::LocalPath>(std::move(envelope));

          std::string data = msg.data();
          uint32_t length = msg.length();
          Eigen::MatrixXf path(length, 2);
          if (msg.length() != 0)
          {
            for (uint32_t i = 0; i < length; i++) {
              float x;
              float y;

              memcpy(&x, data.c_str() + (3 * i + 0) * 4, 4);
              memcpy(&y, data.c_str() + (3 * i + 1) * 4, 4);
              // z not parsed, since not used

              path(i, 0) = x;
              path(i, 1) = y;
            }

            velocityControl.setPath(path);

            // Update and send velocity when we receive new path
            opendlv::proxy::GroundSpeedRequest speedRequest = velocityControl.step();
            cluon::data::TimeStamp sampleTime = cluon::time::now();
            od4.send(speedRequest, sampleTime, 2201);
          
            if (VERBOSE) {
              std::cout << "Ground speed request: " << speedRequest.groundSpeed() << std::endl; 
            }
          }


        }
      }};
    od4.dataTrigger(opendlv::logic::action::LocalPath::ID(), onLocalPath);

    // Just sleep as this microservice is data driven
    using namespace std::literals::chrono_literals;
    while(od4.isRunning()) {
      std::this_thread::sleep_for(1s);
    }

  }
  return retCode;
}

