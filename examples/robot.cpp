//
// Created by keunjun on 24. 10. 26.
//

#include <iostream>

#include "rby1a/integrated_robot.h"

using namespace std::chrono_literals;
using namespace rb::y1a;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config file>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};

  IntegratedRobot robot(config_file);
  if (!robot.WaitUntilReady(10s)) {
    return 1;
  }

  auto obs = robot.GetObservation();
  IntegratedRobot::Action action;
  action.actions.block<2, 1>(0, 0).setZero();                  // Mobility
  action.actions.block<6 + 7 + 7, 1>(2, 0).setZero();          // Body (torso + right arm + left arm)
  action.actions.block<2, 1>(2 + 6 + 7 + 7, 0).setZero();      // Head
  action.actions.block<2, 1>(2 + 6 + 7 + 7 + 2, 0).setZero();  // Gripper
  robot.Step(action, 5);

  std::this_thread::sleep_for(5s);

  return 0;
}