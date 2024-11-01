//
// Created by keunjun on 24. 10. 26.
//

#include <iostream>
#include <queue>

#include "nlohmann/json.hpp"
#include "rby1a/integrated_robot.h"
#include "utils.h"
#include "zmq.hpp"

using namespace std::chrono_literals;
using namespace rb::y1a;
using namespace nlohmann;

zmq::context_t zmq_ctx;
zmq::socket_t sock_;

struct TimedAction {
  long timestamp{0};
  double weight;
  IntegratedRobot::Action action;
};

struct comp {
  bool operator()(const TimedAction& l, const TimedAction& r) { return l.timestamp > r.timestamp; }
};

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <config file> <inference address>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};
  std::string inference_address{argv[2]};

  sock_ = zmq::socket_t(zmq_ctx, zmq::socket_type::req);
  sock_.connect(inference_address);

  IntegratedRobot robot(config_file);
  if (!robot.WaitUntilReady(10s)) {
    return 1;
  }

  while (true) {
    char c;
    std::cout << "Do you want to start? (y/n/z/r)" << std::endl;
    std::cin >> c;
    if (c == 'y') {
      break;
    }
    if (c == 'z') {
      IntegratedRobot::Action action;
      action.actions.setZero();
      robot.Step(action, 5);
    }
    if (c == 'r') {
      auto obs = robot.GetObservation();
      IntegratedRobot::Action action;
      action.actions.block<2, 1>(0, 0).setZero();                  // Mobility
      action.actions.block<20, 1>(2, 0) << 0, 40, -70, 30, 0, 0,   // Torso
          -30, -10, 0, -100, 0, 40, 0,                             // Right
          -30, 10, 0, -100, 0, 40, 0;                              // Left
      action.actions.block<2, 1>(2 + 20, 0) << 0, 0.8;             // Head
      action.actions.block<2, 1>(2 + 6 + 7 + 7 + 2, 0).setZero();  // Gripper
      action.actions *= M_PI / 180.;
      robot.Step(action, 5);
    }
  }

  for (int i = 0; i < 6; i++) {
    std::cout << 6 - i << " ";
    std::flush(std::cout);
    std::this_thread::sleep_for(1s);
  }

  /*****************************************************************
   *
   *****************************************************************/

  auto rt_thread = std::make_unique<rb::Thread>();
  rt_thread->SetOSPriority(90, SCHED_RR);
  rb::EventLoop loop(std::move(rt_thread));

  std::future<std::pair<long, std::vector<rb::y1a::IntegratedRobot::Action>>> inference_future;
  std::vector<std::pair<long, rb::y1a::IntegratedRobot::Action>> inference_result;

  std::queue<TimedAction> que;

  rb::y1a::IntegratedRobot::Action action;
  rb::y1a::IntegratedRobot::Action target;
  action.actions.setZero();
  {
    auto obs = robot.GetObservation();
    action.actions.setZero();
    action.actions.head<rb::y1a::IntegratedRobot::kRobotDOF>() = obs.robot_qpos;
    action.actions.tail<rb::y1a::IntegratedRobot::kGripperDOF>() = obs.gripper_qpos;
  }
  target = action;

  auto fps = 50.;
  auto dt = 1. / fps;
  auto dt_micro = (long)(dt * 1e6);
  auto lpf_gain = 0.1;
  loop.PushCyclicTask(
      [&] {
        auto obs = robot.GetObservation();
        auto current_time =
            std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch())
                .count();

        bool done = true;
        if (inference_future.valid()) {
          if (inference_future.wait_for(0s) == std::future_status::timeout) {
            done = false;
          } else if (inference_future.wait_for(0s) == std::future_status::ready) {
            auto inference_result = inference_future.get();
            long start = inference_result.first;
            for (int i = 0; i < inference_result.second.size(); i++) {
              TimedAction ta;
              ta.timestamp = start + dt_micro * (i + 1);
              ta.weight = (double)(inference_result.second.size() - i) / (double)inference_result.second.size();
              ta.action = inference_result.second[i];
              // if (ta.timestamp >= current_time) {
              que.push(ta);
              // }
            }
          }
        }
        if (done && que.empty()) {
          inference_future = std::async(std::launch::async, [=] {
            auto gripper = obs.gripper_qpos;

            gripper(0) = gripper(0) * (13.522 - 3.98988) + 3.98988;
            gripper(1) = gripper(1) * (15.1726 - 5.59596) + 5.59596;

            {
              json j;
              j["timestamp"] = current_time;
              j["robot_qpos"] = obs.robot_qpos;
              j["gripper_qpos"] = gripper;
              for (const auto& [name, image] : obs.images) {
                j[name] = MatToJson(image);
              }
              sock_.send(zmq::buffer(json::to_msgpack(j)));
            }

            {
              zmq::message_t msg;
              auto rv = sock_.recv(msg, zmq::recv_flags::none);
              std::vector<uint8_t> vec;
              vec.resize(msg.size());
              std::memcpy(vec.data(), msg.data(), msg.size());
              auto j = json::from_msgpack(vec);
              long timestamp = j.value("timestamp", (long)0);
              std::vector<IntegratedRobot::Action> actions;
              for (const auto& arr : j["action"]) {
                IntegratedRobot::Action act;
                if (act.actions.size() != arr.size()) {
                  continue;
                }
                for (int i = 0; i < arr.size(); i++) {
                  act.actions[i] = arr[i].template get<double>();
                }
                actions.push_back(act);
              }

              return std::make_pair(timestamp, actions);
            }
          });
        }

        double weight_sum{0};
        rb::y1a::IntegratedRobot::Action ref;
        ref.actions.setZero();
        if (false) {
          while (!que.empty()) {
            auto item = que.front();
            if (item.timestamp <= current_time) {
              ref.actions += item.weight * item.action.actions;
              weight_sum += item.weight;
            } else {
              break;
            }
            que.pop();
          }
        }

        if (true) {
          while (!que.empty()) {
            auto item = que.front();

            ref.actions += item.weight * item.action.actions;
            weight_sum += item.weight;

            que.pop();
            break;
          }
        }

        if (weight_sum > 0) {
          ref.actions /= weight_sum;

          ref.actions(2 + 6 + 7 + 7 + 2 + 0) = (ref.actions(2 + 6 + 7 + 7 + 2 + 0) - 3.98988) / (13.522 - 3.98988);
          ref.actions(2 + 6 + 7 + 7 + 2 + 1) = (ref.actions(2 + 6 + 7 + 7 + 2 + 1) - 5.59596) / (15.1726 - 5.59596);

          target = ref;
        }

        Eigen::Vector<double, 2 + 6> tmp = action.actions.block<2 + 6, 1>(0, 0);
        action.actions = lpf_gain * target.actions + (1 - lpf_gain) * action.actions;
        action.actions.tail<2>() = target.actions.tail<2>();
        action.actions.block<2 + 6, 1>(0, 0) = tmp;

        robot.Step(action, 1. / fps * 1.01);
      },
      std::chrono::nanoseconds((long)(1.e9 / fps)));

  while (true) {
    std::this_thread::sleep_for(1s);
  }

  return 0;
}