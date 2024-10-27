#include "app_main.h"

#include <filesystem>
#include <string>

#include "nlohmann/json.hpp"
#include "zmq_addon.hpp"

namespace {
nlohmann::json MatToJson(const cv::Mat& mat) {
  nlohmann::json j;
  j["rows"] = mat.rows;
  j["cols"] = mat.cols;
  j["type"] = mat.type();

  std::vector<uint8_t> data;

  if (mat.isContinuous()) {
    data.resize(mat.total() * mat.elemSize());
    std::memcpy(data.data(), mat.data, mat.total() * mat.elemSize());
  } else {
    throw std::runtime_error("cv::Mat is not continuous");
  }

  // Add data to JSON object as a Base64 encoded string or raw data vector
  j["data"] = data;

  return j;
}
}  // namespace

AppMain::AppMain(const std::string& config_file) {
  using namespace toml;

  Config config;
  auto tbl = toml::parse_file(config_file);

  std::stringstream master_ss;
  master_ss << tbl["master"];
  config.master_config = master_ss.str();

  std::stringstream slave_ss;
  slave_ss << tbl["slave"];
  config.slave_config = slave_ss.str();

  ConfigRecord& config_record = config.record;
  auto record = tbl["record"];
  config_record.fps = record["fps"].value_or(config_record.fps);
  config_record.path = record["path"].value_or(config_record.path);

  ConfigServer& config_server = config.server;
  auto server = tbl["server"];
  config_server.robot_address = server["robot_address"].value_or(config_server.robot_address);
  config_server.request_server_port = server["request_server_port"].value_or(config_server.request_server_port);
  config_server.publisher_server_port = server["publisher_server_port"].value_or(config_server.publisher_server_port);

  Initialize(config);
}

AppMain::~AppMain() {
  if (slave_) {
    slave_.reset();
  }

  if (master_) {
    master_.reset();
  }
}

void AppMain::Initialize(const Config& config) {
  using namespace rb;
  using namespace std::chrono_literals;

  config_ = config;

  //

  master_ = std::make_unique<y1a::MasterArm>(y1a::MasterArm::ParseConfig(config_.master_config));

  slave_ = std::make_unique<y1a::IntegratedRobot>(y1a::IntegratedRobot::ParseConfig(config_.slave_config));
  if (!slave_->WaitUntilReady(10s)) {
    std::cout << "error" << std::endl;
    slave_.reset();
  }

  //  InitializeRecord();

  InitializeServer();
}

void AppMain::InitializeServer() {
  using namespace rb;
  using namespace std::chrono_literals;

  robot_ = Robot<y1_model::A>::Create(config_.server.robot_address);
  if (!robot_->Connect(1 /* iteration */, 1000 /* (ms) */)) {
    throw std::runtime_error("failed to connect robot");
  }

  int power_12v_idx = -1;
  int power_48v_idx = -1;

  auto robot_info = robot_->GetRobotInfo();
  for (int i = 0; i < robot_info.power_infos.size(); i++) {
    const auto& name = robot_info.power_infos[i].name;
    if (name == "12v") {
      power_12v_idx = i;
    } else if (name == "48v") {
      power_48v_idx = i;
    }
  }
  if (power_12v_idx < 0 || power_48v_idx < 0) {
    throw std::runtime_error("failed to retrieve index of power");
  }

  robot_->StartStateUpdate(
      [=](const RobotState<y1_model::A>& state, const ControlManagerState& control_state) {
        state_buf_.PushTask([=] {
          state_.power_12v = (state.power_states[power_12v_idx].state == PowerState::State::kPowerOn);
          state_.power_48v = (state.power_states[power_48v_idx].state == PowerState::State::kPowerOn);
          state_.servo_on = state.is_ready.all();
          state_.control_manger = (control_state.state == ControlManagerState::State::kEnabled &&
                                   (control_state.control_state == ControlManagerState::ControlState::kExecuting ||
                                    control_state.control_state == ControlManagerState::ControlState::kSwitching));
        });
      },
      10 /* (Hz) */);

  robot_dyn_ = robot_->GetDynamics();
  robot_dyn_state_ =
      robot_dyn_->MakeState({"base", "link_head_0", "ee_right", "ee_left"}, y1_model::A::kRobotJointNames);
  robot_dyn_state_->SetGravity({0, 0, 0, 0, 0, -9.81});

  q_upper_limit_ = robot_dyn_->GetLimitQUpper(robot_dyn_state_);
  q_lower_limit_ = robot_dyn_->GetLimitQLower(robot_dyn_state_);

  pub_sock_ = zmq::socket_t(zmq_ctx_, zmq::socket_type::pub);
  pub_sock_.bind("tcp://*:" + std::to_string(config_.server.publisher_server_port));

  srv_sock_ = zmq::socket_t(zmq_ctx_, zmq::socket_type::router);
  srv_sock_.bind("tcp://*:" + std::to_string(config_.server.request_server_port));

  publisher_ev_.PushCyclicTask(
      [=] {
        using namespace nlohmann;

        state_buf_.PushTask([=] {
          std::filesystem::space_info si = std::filesystem::space(config_.record.path);
          state_.upc_storage_free = (double)si.free / (1 << 20);  // MB 단위로 변환
          state_.upc_storage_available = (double)si.available / (1 << 20);
          state_.upc_storage_capacity = (double)si.capacity / (1 << 20);

          state_.master_state = master_->GetState();
          state_.slave_observation = slave_->GetObservation();
        });

        auto state = state_buf_.DoTask([=] { return state_; });

        json j;
        j["power_12v"] = state.power_12v;
        j["power_48v"] = state.power_48v;
        j["servo_on"] = state.servo_on;
        j["control_manger"] = state.control_manger;
        j["teleop"] = state.teleop;
        j["recording"] = state.recording;
        j["recording_count"] = state.recording_count;
        j["storage_available"] = state_.upc_storage_available;
        j["storage_free"] = state_.upc_storage_free;
        j["storage_capacity"] = state_.upc_storage_capacity;
        zmq::send_multipart(pub_sock_,
                            std::array<zmq::const_buffer, 2>{zmq::str_buffer("data"), zmq::buffer(j.dump())});

        static auto last_time = std::chrono::steady_clock::now();
        if (std::chrono::steady_clock::now() - last_time > 500ms) {
          nlohmann::json image_j;
          for (const auto& [name, image] : state.slave_observation.images) {
            image_j[name] = MatToJson(image);
          }
          zmq::send_multipart(pub_sock_,
                              std::array<zmq::const_buffer, 2>{zmq::str_buffer("image"), zmq::buffer(image_j.dump())});
          last_time = std::chrono::steady_clock::now();
        }
      },
      100ms);

  /************************
   * SERVICE
   ************************/

  service_ev_.PushCyclicTask(
      [=] {
        State state = state_buf_.DoTask([=] { return state_; });

        try {
          std::vector<zmq::message_t> recv_msgs;
          zmq::recv_result_t result =
              zmq::recv_multipart(srv_sock_, std::back_inserter(recv_msgs), zmq::recv_flags::dontwait);
          if (!result.has_value() || recv_msgs.empty()) {
            return;
          }

          auto j = nlohmann::json::parse(recv_msgs[1].to_string());
          if (!j.contains("command")) {
            return;
          }

          std::string command = j["command"];
          std::cout << "[Service] command received: " << command << std::endl;

          if (command == kCommandResetMaster) {
            if (!state.teleop) {
              master_ = std::make_unique<y1a::MasterArm>(y1a::MasterArm::ParseConfig(config_.master_config));
            }
          } else if (command == kCommandResetSlave) {
            if (!state.teleop) {
              slave_ = std::make_unique<y1a::IntegratedRobot>(y1a::IntegratedRobot::ParseConfig(config_.slave_config));
            }
          } else if (command == kCommandZeroPose) {
            if (slave_ && !state.teleop) {
              y1a::IntegratedRobot::Action action;
              action.actions.setZero();
              slave_->Step(action, 5);
            }
          } else if (command == kCommandReadyPose) {
            if (slave_ && !state.teleop) {
              y1a::IntegratedRobot::Action action;
              action.actions.setZero();
              action.actions.block<20, 1>(2, 0) << 0, 40, -70, 30, 0, 0,  // Torso
                  -30, -10, 0, -100, 0, 40, 0,                            // Right
                  -30, 10, 0, -100, 0, 40, 0;                             // Left
              action.actions.block<2, 1>(2 + 20, 0) << 0, 0;              // Head
              action.actions *= M_PI / 180.;
              slave_->Step(action, 5);
            }
          } else if (command == kCommandStartTeleop) {
            teleop_ = std::make_unique<Teleop>(this);
          } else if (command == kCommandStopTeleop) {
            teleop_.reset();
          }
        } catch (...) {}
      },
      10ms);
}

void AppMain::Wait() {
  service_ev_.WaitForTasks();
}

AppMain::Teleop::Teleop(AppMain* app) : app_(app) {
  app_->state_.teleop = true;

  std::cout << "Construct tele-operation" << std::endl;

  auto slave_obs = app_->slave_->GetObservation();
  qpos_ref_ = slave_obs.robot_qpos;

  loop_.PushCyclicTask([=] { loop(); }, std::chrono::nanoseconds((long)(1.e9 / app_->config_.record.fps)));
}

AppMain::Teleop::~Teleop() {
  // app_->StopRecording();
  app_->state_.teleop = false;

  std::cout << "Destruct tele-operation" << std::endl;
}

void AppMain::Teleop::loop() {
  const double LpfGain = 0.2;
  auto slave_obs = app_->slave_->GetObservation();
  auto master_state = app_->master_->GetState();

  if (master_state.button_right.button) {
    right_ratio_ = std::min(right_ratio_ + LpfGain / app_->config_.record.fps, LpfGain);
  } else {
    right_ratio_ = 0;
  }
  if (master_state.button_left.button) {
    left_ratio_ = std::min(left_ratio_ + LpfGain / app_->config_.record.fps, LpfGain);
  } else {
    left_ratio_ = 0;
  }
  qpos_ref_.block<7, 1>(2 + 6, 0) =
      qpos_ref_.block<7, 1>(2 + 6, 0) * (1 - right_ratio_) + master_state.q_joint.block<7, 1>(0, 0) * right_ratio_;
  qpos_ref_.block<7, 1>(2 + 6 + 7, 0) =
      qpos_ref_.block<7, 1>(2 + 6 + 7, 0) * (1 - left_ratio_) + master_state.q_joint.block<7, 1>(7, 0) * left_ratio_;

  //
  app_->robot_dyn_state_->SetQ(slave_obs.robot_qpos);
  app_->robot_dyn_->ComputeForwardKinematics(app_->robot_dyn_state_);
  auto T_12 = app_->robot_dyn_->ComputeTransformation(app_->robot_dyn_state_, 1, 2);
  auto T_13 = app_->robot_dyn_->ComputeTransformation(app_->robot_dyn_state_, 1, 3);
  Eigen::Vector3d center = (rb::math::SE3::GetPosition(T_12) + rb::math::SE3::GetPosition(T_13)) / 2.;
  double yaw = atan2(center(1), center(0));
  double pitch = atan2(-center(2), center(0)) + 15 * M_PI / 180.;
  yaw = std::clamp(yaw, -0.523, 0.523);  // TODO
  pitch = std::clamp(pitch, -0.35, 1.57);
  qpos_ref_.block<2, 1>(2 + 6 + 7 + 7, 0) = Eigen::Vector<double, 2>{yaw, pitch};
  for (int i = 0; i < 14 + 2; i++) {
    qpos_ref_(2 + 6 + i) =
        std::clamp(qpos_ref_(2 + 6 + i), app_->q_lower_limit_(2 + 6 + i), app_->q_upper_limit_(2 + 6 + i));
  }

  rb::y1a::IntegratedRobot::Action action;
  action.actions.head<rb::y1a::IntegratedRobot::kRobotDOF>() = qpos_ref_;
  action.actions.tail<rb::y1a::IntegratedRobot::kGripperDOF>() =
      Eigen::Vector2d::Constant(1.) -
      Eigen::Vector2d{master_state.button_right.trigger, master_state.button_left.trigger} / 1000.;
  app_->slave_->Step(action, 1 / app_->config_.record.fps * 1.05);

  // app_->Record(slave_obs, action);
}