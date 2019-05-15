/*
 * Copyright (C) 2018  Christian Berger
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

#include "logic-velocity.hpp"
#include <iostream>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if (0 == commandlineArguments.count("cid")) {
        std::cerr << argv[0] << "Generates the speed requests for Lynx" << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> [--verbose=<Verbose or not>]"
        << std::endl;
        std::cerr << "Example: " << argv[0] << "--cid=111 [--verbose]" << std::endl;
        retCode = 1;
    } else {

        // Interface to a running OpenDaVINCI session.  
        cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

        // Grab command line arguments
        bool VERBOSE{static_cast<bool>(commandlineArguments.count("verbose"))};

        // VelocityControl object plans the speed
        VelocityControl velocityControl;

        // Update on new aimpoint data
        auto onAimPoint{[&velocityControl, &od4, VERBOSE](cluon::data::Envelope &&envelope)
        {
          uint16_t senderStamp = envelope.senderStamp();
          if (senderStamp == 1904) {
            auto aimPoint = cluon::extractMessage<opendlv::logic::action::AimPoint>(std::move(envelope));
            velocityControl.setAimPoint(aimPoint);

            // Update and send velocity when we receive new aimpoint
            auto speedRequest = velocityControl.step();
            cluon::data::TimeStamp sampleTime = cluon::time::now();
            od4.send(speedRequest, sampleTime, 2201);

            if (VERBOSE) {
              std::cout << "[COGNITION-VELOCITY] Aim point distance: " << aimPoint.distance() << "\n"
                        << "[COGNITION-VELOCITY] Aim point azimuth angle: " << aimPoint.azimuthAngle() << std::endl;
            }
          }
        }};
        od4.dataTrigger(opendlv::logic::action::AimPoint::ID(), onAimPoint);


        // Just sleep as this microservice is data driven
        using namespace std::literals::chrono_literals;
        while(od4.isRunning()) {
          std::this_thread::sleep_for(1s);
        }

    }
    return retCode;
}

