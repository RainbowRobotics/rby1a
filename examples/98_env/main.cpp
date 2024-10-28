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

  while (true) {
    char c;
    std::cout << "Do you want to start? (y/n)" << std::endl;
    std::cin >> c;
    if (c == 'y') {
      break;
    }
  }

  while (true) {
    char c;
    std::cout << "Go to ready pose? (y/n)" << std::endl;
    std::cin >> c;
    if (c == 'y') {
      break;
    }
  }

  auto obs = robot.GetObservation();
  IntegratedRobot::Action action;
  action.actions.block<2, 1>(0, 0).setZero();                  // Mobility
  action.actions.block<20, 1>(2, 0) << 0, 40, -70, 30, 0, 0,  // Torso
      -30, -10, 0, -100, 0, 40, 0,                            // Right
      -30, 10, 0, -100, 0, 40, 0;                             // Left
  action.actions.block<2, 1>(2 + 20, 0) << 0, 0;              // Head
  action.actions.block<2, 1>(2 + 6 + 7 + 7 + 2, 0).setZero();  // Gripper
  action.actions *= M_PI / 180.;
  robot.Step(action, 5);

  std::this_thread::sleep_for(5s);

  /*****************************************************************
   *
   *****************************************************************/

  return 0;
}