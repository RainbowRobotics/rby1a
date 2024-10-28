//
// Created by keunjun on 24. 10. 26.
//

#include <iostream>

#include "rby1a/integrated_robot.h"
#include "zmq.hpp"

#define RBY1A_DEBUG

using namespace std::chrono_literals;
using namespace rb::y1a;

zmq::context_t zmq_ctx;
zmq::socket_t sock_;

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <config file> <inference address>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};
  std::string inference_address{argv[2]};

  sock_ = zmq::socket_t(zmq_ctx, zmq::socket_type::client);
  sock_.bind(inference_address);

  IntegratedRobot robot(config_file);
  if (!robot.WaitUntilReady(10s)) {
    return 1;
  }

  while (true) {
    char c;
    std::cout << "Do you want to start? (y/n/z)" << std::endl;
    std::cin >> c;
    if (c == 'y') {
      break;
    }
    if (c == 'z') {
#ifndef RBY1A_DEBUG
      IntegratedRobot::Action action;
      action.actions.setZero();
      robot.Step(action, 5);
#endif
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
#ifndef RBY1A_DEBUG
  IntegratedRobot::Action action;
  action.actions.block<2, 1>(0, 0).setZero();                  // Mobility
  action.actions.block<20, 1>(2, 0) << 0, 40, -70, 30, 0, 0,   // Torso
      -30, -10, 0, -100, 0, 40, 0,                             // Right
      -30, 10, 0, -100, 0, 40, 0;                              // Left
  action.actions.block<2, 1>(2 + 20, 0) << 0, 0;               // Head
  action.actions.block<2, 1>(2 + 6 + 7 + 7 + 2, 0).setZero();  // Gripper
  action.actions *= M_PI / 180.;
  robot.Step(action, 5);
#endif

  std::this_thread::sleep_for(5s);

  /*****************************************************************
   *
   *****************************************************************/

  auto rt_thread = std::make_unique<rb::Thread>();
  rt_thread->SetOSPriority(90, SCHED_RR);
  rb::EventLoop loop(std::move(rt_thread));

//  std::future<void>

  auto fps = 50.;
  auto dt = 1. / fps;
  loop.PushCyclicTask(
      [&] {

      },
      std::chrono::nanoseconds((long)(1.9 / fps)));

  return 0;
}