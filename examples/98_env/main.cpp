//
// Created by keunjun on 24. 10. 26.
//

#include <iostream>
#include <queue>
#include <csignal>

#include "highfive/H5Easy.hpp"
#include "nlohmann/json.hpp"
#include "rby1a/integrated_robot.h"
#include "utils.h"
#include "zmq.hpp"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace rb::y1a;
using namespace nlohmann;

zmq::context_t zmq_ctx;
zmq::socket_t sock_;

std::unique_ptr<rb::EventLoop> loop;
std::unique_ptr<HighFive::File> logger;

template <typename T>
static void AddDataIntoDataSet(HighFive::DataSet& dataset, const std::vector<std::size_t>& shape, T* data) {
  auto current_dims = dataset.getSpace().getDimensions();
  size_t current_rows = current_dims[0];
  size_t new_rows = current_rows + 1;
  std::vector<std::size_t> resize = {new_rows}, offset = {current_rows}, count = {1};
  for (const auto& l : shape) {
    resize.push_back(l);
    offset.push_back(0);
    count.push_back(l);
  }
  dataset.resize(resize);
  dataset.select(offset, count).write_raw(data);
}

template <typename T>
static HighFive::DataSet CreateDataSet(HighFive::File& file, const std::string& name,
                                       const std::vector<std::size_t>& shape, int compression_level = 0) {
  std::vector<size_t> dims = {0};
  std::vector<size_t> max_dims = {HighFive::DataSpace::UNLIMITED};
  std::vector<hsize_t> chunk_size = {1};
  for (const auto& l : shape) {
    dims.push_back(l);
    max_dims.push_back(l);
    chunk_size.push_back(l);
  }
  HighFive::DataSetCreateProps props;
  props.add(HighFive::Chunking(chunk_size));
  if (compression_level != 0) {
    props.add(HighFive::Deflate(compression_level));
  }
  return file.createDataSet<T>(name, HighFive::DataSpace(dims, max_dims), props);
}

void signal_handler(int signum) {
  std::cout << "signal handler" << std::endl;

  if (loop) {
    loop.reset();
  }

  if(logger) {
    logger->flush();
    logger.reset();
  }

  _exit(EXIT_FAILURE);
  signal(SIGTERM, SIG_DFL);
  raise(SIGTERM);
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);

  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <config file> <inference address> <log file (default: log.h5)>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};
  std::string inference_address{argv[2]};
  std::string log_file{"log.h5"};
  if (argc >= 4) {
    log_file = std::string{argv[3]};
  }

  sock_ = zmq::socket_t(zmq_ctx, zmq::socket_type::req);
  sock_.connect(inference_address);

  logger = std::make_unique<HighFive::File>(log_file, HighFive::File::Overwrite);
  HighFive::DataSet log_action = CreateDataSet<double>(*logger, "/action", {26});
  HighFive::DataSet log_last_obs = CreateDataSet<double>(*logger, "/obs_qpos", {26});
  HighFive::DataSet log_qpos_ref = CreateDataSet<double>(*logger, "/qpos_ref", {26});
  HighFive::DataSet log_qpos = CreateDataSet<double>(*logger, "/qpos", {26});

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
      action.actions.block<2, 1>(2 + 20, 0) << 0, 45;             // Head
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
  loop = std::make_unique<rb::EventLoop>(std::move(rt_thread));

  std::future<std::pair<long, std::vector<IntegratedRobot::Action>>> inference_future;
  std::vector<std::pair<long, IntegratedRobot::Action>> inference_result;

  std::queue<IntegratedRobot::Action> que;

  IntegratedRobot::Action action;
  IntegratedRobot::Action last_inference_action;
  IntegratedRobot::Observation last_obs;
  action.actions.setZero();
  {
    auto obs = robot.GetObservation();
    last_obs = obs;
    action.actions.setZero();
    action.actions.head<IntegratedRobot::kRobotDOF>() = obs.robot_qpos;
    action.actions.tail<IntegratedRobot::kGripperDOF>() = obs.gripper_qpos;
  }
  last_inference_action = action;

  auto fps = 50.;
  auto dt = 1. / fps;
  auto lpf_gain = 0.1;
  loop->PushCyclicTask(
      [&] {
        auto obs = robot.GetObservation();
        obs.gripper_qpos(0) = obs.gripper_qpos(0) * (13.522 - 3.98988) + 3.98988;
        obs.gripper_qpos(1) = obs.gripper_qpos(1) * (15.1726 - 5.59596) + 5.59596;

        auto current_time = duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();

        bool done = true;
        if (inference_future.valid()) {
          if (inference_future.wait_for(0s) == std::future_status::timeout) {
            done = false;
          } else if (inference_future.wait_for(0s) == std::future_status::ready) {
            auto inference_result = inference_future.get();
            long start = inference_result.first;
            // for (const auto& item : inference_result.second) {
            //   que.push(item);
            // }
            for (int i = 0; i < inference_result.second.size(); i++) {
              que.push(inference_result.second[i]);
            }
          }
        }
        if (done && que.empty()) {
          inference_future = std::async(std::launch::async, [=, &last_obs] {
            last_obs = obs;
            auto gripper = obs.gripper_qpos;

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
              auto j = json::from_msgpack(vec);  // Parsing

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
                const int idx = 2 + 6 + 7 + 7 + 2;
                act.actions(idx + 0) = (act.actions(idx + 0) - 3.98988) / (13.522 - 3.98988);
                act.actions(idx + 1) = (act.actions(idx + 1) - 5.59596) / (15.1726 - 5.59596);

                actions.push_back(act);
              }

              return std::make_pair(timestamp, actions);
            }
          });
        }

        if (!que.empty()) {
          auto item = que.front();
          que.pop();
          last_inference_action = item;
        }

        IntegratedRobot::Action prev_action = action;
        // LPF
        action.actions = lpf_gain * last_inference_action.actions + (1 - lpf_gain) * action.actions;
        // Keep position of wheel and torso
        action.actions.head<2 + 6>() = prev_action.actions.head<2 + 6>();

        /*****************
         * LOGGING
         *****************/
        AddDataIntoDataSet(log_action, {26}, last_inference_action.actions.data());
        AddDataIntoDataSet(log_qpos_ref, {26}, action.actions.data());
        Eigen::Vector<double, 26> qpos;
        qpos.head<24>() = obs.robot_qpos;
        qpos.tail<2>() = obs.gripper_qpos;
        AddDataIntoDataSet(log_qpos, {26}, qpos.data());
        Eigen::Vector<double, 26> last_obs_qpos;
        last_obs_qpos.head<24>() = last_obs.robot_qpos;
        last_obs_qpos.tail<2>() = last_obs.gripper_qpos;
        AddDataIntoDataSet(log_last_obs, {26}, last_obs_qpos.data());

        // action.actions[2 + 6 + 7 + 7 + 0] = 0;
        // action.actions[2 + 6 + 7 + 7 + 1] = 45 * M_PI / 180;

        robot.Step(action, 1. / fps * 1.01);
      },
      nanoseconds((long)(1.e9 / fps)));

  while (true) {
    std::this_thread::sleep_for(1s);
  }

  return 0;
}