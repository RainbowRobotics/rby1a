#include <utility>

#include "rby1-sdk/upc/device.h"
#include "rby1a/integrated_robot.h"
#include "toml++/toml.hpp"

using namespace std::chrono_literals;

namespace {
rs2::context rs_ctx;

std::string ReadFileToString(const std::string& filePath) {
  std::ifstream file(filePath);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open file: " + filePath);
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  return buffer.str();
}
}  // namespace

namespace rb::y1a {

IntegratedRobot::Config IntegratedRobot::ParseConfig(const std::string& toml_str) {
  using namespace toml;

  Config config;
  auto tbl = toml::parse(toml_str);

  ConfigRobot& config_robot = config.robot;
  auto robot = tbl["robot"];
  config_robot.address = robot["address"].value_or(config_robot.address);
  if (auto robot_action = robot["action"].as_table()) {
    config_robot.act_mobility = robot_action->at("mobility").value_or(config_robot.act_mobility);
  }
  if (auto robot_observation = robot["observation"].as_table()) {
    config_robot.obs_torque = robot_observation->at("torque").value_or(config_robot.obs_torque);
    config_robot.obs_ft = robot_observation->at("ft").value_or(config_robot.obs_ft);
  }

  ConfigGripper& config_gripper = config.gripper;
  auto gripper = tbl["gripper"];
  config_gripper.dev_name = gripper["dev_name"].value_or(config_gripper.dev_name);

  ConfigCamera& config_camera = config.camera;
  auto camera = tbl["camera"];
  config_camera.height = camera["height"].value_or(config_camera.height);
  config_camera.width = camera["width"].value_or(config_camera.width);
  config_camera.fps = camera["fps"].value_or(config_camera.fps);
  if (toml::array* arr = camera["sensors"].as_array()) {
    arr->for_each([&](const auto& el) {
      if constexpr (toml::is_table<decltype(el)>) {
        ConfigCamera::Sensor sensor;

        sensor.name = el.at("name").value_or(sensor.name);
        sensor.serial = el.at("serial").value_or(sensor.serial);

        if (sensor.name.empty()) {
          throw std::invalid_argument("sensor.name invalid argument");
        }
        if (sensor.serial.empty()) {
          throw std::invalid_argument("sensor.serial invalid argument");
        }

        config.camera.sensors.push_back(sensor);
      }
    });
  }

  return config;
}

IntegratedRobot::IntegratedRobot(const Config& config) {
  Initialize(config);
}

IntegratedRobot::IntegratedRobot(const std::string& config_file)
    : IntegratedRobot(ParseConfig(ReadFileToString(config_file))) {}

IntegratedRobot::~IntegratedRobot() {
  camera_loop_.reset();
  std::cout << "reset camera loop" << std::endl;

  gripper_loop_.reset();
  std::cout << "reset gripper loop" << std::endl;

  if (robot_command_stream_handler_) {
    std::cout << "try to cancel robot command stream ...";
    robot_command_stream_handler_->Cancel();
    robot_command_stream_handler_->Wait();
    robot_command_stream_handler_.reset();
    std::cout << " - finished" << std::endl;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  std::cout << "disable control manager" << std::endl;
  robot_->DisableControlManager();

  gripper_.reset();
  robot_.reset();

  //  robot_->PowerOff("48v");
}

void IntegratedRobot::Initialize(const Config& config) {
  config_ = config;

  /********************************
   * Robot
   ********************************/
  Initialize_robot();

  /********************************
   * Gripper
   ********************************/
  Initialize_gripper();

  /********************************
   * Camera
   ********************************/
  Initialize_camera();
}

void IntegratedRobot::Initialize_robot() {
  robot_ = Robot<Model>::Create(config_.robot.address);
  if (!robot_->Connect(1 /* iteration */, 1000 /* (ms) */)) {
    throw std::runtime_error("failed to connect robot");
  }
  std::cout << "robot connected ..." << std::endl;

  // power on & enable control manager
  if (!robot_->IsPowerOn("48v")) {
    if (!robot_->PowerOn("48v")) {
      throw std::runtime_error("failed to power on");
    }
  }
  std::this_thread::sleep_for(500ms);
  std::cout << "power on" << std::endl;
  if (!robot_->IsServoOn(".*")) {
    if (!robot_->ServoOn(".*")) {
      throw std::runtime_error("failed to servo on");
    }
  }
  std::this_thread::sleep_for(500ms);
  std::cout << "servo on" << std::endl;
  robot_->ResetFaultControlManager();
  if (!robot_->EnableControlManager()) {
    throw std::runtime_error("failed to enable control manager");
  }
  std::this_thread::sleep_for(500ms);
  std::cout << "control manager enabled" << std::endl;
  robot_->StartStateUpdate(
      [=](const RobotState<Model>& rs) {
        gripper_power_state_.store(rs.tool_flange_left.output_voltage == 12 &&
                                   rs.tool_flange_right.output_voltage == 12);

        auto current_time = std::chrono::steady_clock::now();
        observation_buf_.PushTask([=] {
          obs_state.robot_updated_time = current_time;
          obs.robot_qpos = rs.position;
          obs.robot_qvel = rs.velocity;
          if (config_.robot.obs_torque) {
            obs.robot_torque = rs.torque;
          }
          if (config_.robot.obs_ft) {
            obs.right_ft.head<3>() = rs.ft_sensor_right.torque;
            obs.right_ft.tail<3>() = rs.ft_sensor_right.force;
            obs.left_ft.head<3>() = rs.ft_sensor_left.torque;
            obs.left_ft.tail<3>() = rs.ft_sensor_left.force;
          }
          obs.robot_qpos_ref = rs.target_position;
        });
      },
      100 /* (Hz) */);

  robot_dyn_ = robot_->GetDynamics();
  std::cout << "get model from robot" << std::endl;
  robot_dyn_state_ =
      robot_dyn_->MakeState({"base", "link_head_0", "ee_right", "ee_left"}, y1_model::A::kRobotJointNames);
  std::cout << "robot state is made" << std::endl;
  robot_dyn_state_->SetGravity({0, 0, 0, 0, 0, -9.81});
  q_upper_limit_ = robot_dyn_->GetLimitQUpper(robot_dyn_state_);
  q_lower_limit_ = robot_dyn_->GetLimitQLower(robot_dyn_state_);

  robot_->ResetAllParametersToDefault();
  robot_->SetParameter("joint_position_command.cutoff_frequency", "4.0");
  std::cout << "parameter set" << std::endl;

  // auto state = robot_->GetState();
  robot_command_stream_handler_ = robot_->CreateCommandStream();

  std::cout << "robot initialization finished" << std::endl;
}

void IntegratedRobot::Initialize_gripper() {
  static const std::vector<double> gripper_torque_constant{1.6591, 1.6591};

  try {
    // Latency timer setting
    upc::InitializeDevice(config_.gripper.dev_name);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to initialize device: " << e.what();
    throw std::runtime_error(ss.str());
  }

  gripper_ = std::make_unique<rb::DynamixelBus>(config_.gripper.dev_name);
  if (!gripper_->OpenPort()) {
    throw std::runtime_error("failed to open port for gripper");
  }
  if (!gripper_->SetBaudRate(DynamixelBus::kDefaultBaudrate)) {
    throw std::runtime_error("failed to initialize gripper bus");
  }
  gripper_->SetTorqueConstant(gripper_torque_constant);

  gripper_loop_ = std::make_unique<EventLoop>();
  gripper_loop_->PushCyclicTask(
      [=] {
        if (!gripper_) {
          return;
        }

        using namespace std::chrono;
        using namespace std::chrono_literals;

        static const auto kConnectionPassIter = 20;
        static const double kGripperLpfGain = 0.2;

        static auto step_start_time = steady_clock::now();
        static Eigen::Vector<double, kGripperDOF> gripper_min{Eigen::Vector<double, kGripperDOF>::Constant(100)};
        static Eigen::Vector<double, kGripperDOF> gripper_max{Eigen::Vector<double, kGripperDOF>::Constant(-100)};
        static Eigen::Vector<int, kGripperDOF> target_operation_mode;
        static Eigen::Vector<double, kGripperDOF> target_torque;
        static Eigen::Vector<double, kGripperDOF> target_position;
        static Eigen::Vector<double, kGripperDOF> prev_target_position{Eigen::Vector<double, kGripperDOF>::Zero()};

        if (robot_) {
          robot_->SetToolFlangeOutputVoltage("left", 12);
          robot_->SetToolFlangeOutputVoltage("right", 12);
        }

        if (!gripper_power_state_.load()) {
          gripper_state_ = 0;
          return;
        }

        auto [o, t, p] = gripper_buf_.DoTask([=] {
          return std::make_tuple(gripper_target_operation_mode, gripper_target_torque, gripper_target_position);
        });
        target_operation_mode = o;
        target_torque = t;
        target_position = p.array() * (gripper_max - gripper_min).array() + gripper_min.array();

        auto s = gripper_state_.load();
        switch (s) {
          case 0: {
            static int success = 0;
            for (int id = 0; id < kGripperDOF; id++) {
              if (!gripper_->Ping(id)) {
                success = 0;
                return;
              }
            }
            if (success++ >= kConnectionPassIter) {
              for (int id = 0; id < kGripperDOF; id++) {
                if (!gripper_->SendOperationMode(id, DynamixelBus::kCurrentControlMode))
                  ;
                gripper_->SendTorqueEnable(id, DynamixelBus::kTorqueEnable);
              }

              gripper_state_++;
              step_start_time = steady_clock::now();
            }
            return;
          }
          case 1:
          case 2: {
            target_operation_mode.setConstant(DynamixelBus::kCurrentControlMode);
            target_torque.setConstant(s == 1 ? 1 : -1);
            if (steady_clock::now() - step_start_time > 2s) {
              gripper_state_++;
              step_start_time = steady_clock::now();
            }
            break;
          }
          case 3: {
            target_operation_mode.setConstant(DynamixelBus::kCurrentControlMode);
            target_torque.setConstant(0);
            gripper_state_++;
            std::cout << "gripper is on" << std::endl;
            break;
          }
          default: {
            //
          }
        }

        Eigen::Vector<int, kGripperDOF> operation_mode;
        Eigen::Vector<double, kGripperDOF> q, qvel, torque;

        auto temp_operation_mode_vector = gripper_->BulkReadOperationMode({0, 1});
        if (temp_operation_mode_vector.has_value()) {
          for (auto const& ret : temp_operation_mode_vector.value()) {
            operation_mode[ret.first] = ret.second;
          }
        } else {
          return;
        }

        auto temp_ms_vector = gripper_->BulkReadMotorState({0, 1});
        if (temp_ms_vector.has_value()) {
          for (auto const& ret : temp_ms_vector.value()) {
            q[ret.first] = ret.second.position;
            qvel[ret.first] = ret.second.velocity;
            torque[ret.first] = ret.second.current * gripper_torque_constant[ret.first];
          }
        } else {
          return;
        }

        std::vector<std::pair<int, int>> changed_id_mode;
        std::vector<int> changed_id;

        std::vector<std::pair<int, double>> id_position;
        std::vector<std::pair<int, double>> id_torque;

        for (int i = 0; i < kGripperDOF; i++) {
          if (operation_mode[i] != target_operation_mode[i]) {
            changed_id.push_back(i);
            changed_id_mode.emplace_back(i, target_operation_mode[i]);

            if (target_operation_mode[i] == DynamixelBus::kCurrentBasedPositionControlMode) {
              prev_target_position[i] = q[i];
            }
          } else {
            if (operation_mode[i] == DynamixelBus::kCurrentControlMode) {
              id_torque.emplace_back(i, target_torque[i]);
            } else if (operation_mode[i] == DynamixelBus::kCurrentBasedPositionControlMode) {
              prev_target_position[i] =
                  (1 - kGripperLpfGain) * prev_target_position[i] + kGripperLpfGain * target_position[i];

              id_torque.emplace_back(i, 1.0);
              id_position.emplace_back(i, prev_target_position[i]);
            }
          }
        }

        gripper_min = gripper_min.cwiseMin(q);
        gripper_max = gripper_max.cwiseMax(q);

        gripper_->BulkWriteTorqueEnable(changed_id, 0);
        gripper_->BulkWriteOperationMode(changed_id_mode);
        gripper_->BulkWriteTorqueEnable(changed_id, 1);

        gripper_->BulkWriteSendTorque(id_torque);
        gripper_->BulkWriteSendPosition(id_position);

        auto current_time = std::chrono::steady_clock::now();
        observation_buf_.PushTask([=] {
          obs_state.gripper_updated_time = current_time;
          obs.gripper_qpos = (q.array() - gripper_min.array()) / (gripper_max.array() - gripper_min.array());
          obs.gripper_qvel = qvel;
          if (config_.robot.obs_torque) {
            obs.gripper_torque = torque;
          }
        });
      },
      std::chrono::milliseconds(1000 / 30));
}

void IntegratedRobot::Initialize_camera() {
  std::vector<rs2::pipeline> rs_pipelines;
  std::unordered_map<std::string, double> depth_scales;

  for (auto&& dev : rs_ctx.query_devices()) {
    std::string serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
    bool sensor_found = false;
    for (const auto& s : config_.camera.sensors) {
      if (s.serial == serial) {
        sensor_found = true;
        break;
      }
    }
    if (!sensor_found) {
      continue;
    }

    auto advanced_dev = dev.as<rs400::advanced_mode>();
    auto advanced_sensors = advanced_dev.query_sensors();

    bool depth_found = false;
    bool color_found = false;
    rs2::sensor depth_sensor;
    rs2::sensor color_sensor;
    for (auto&& sensor : advanced_sensors) {
      std::string module_name = sensor.get_info(RS2_CAMERA_INFO_NAME);
      std::cout << module_name << std::endl;

      if (module_name == "Stereo Module") {
        depth_sensor = sensor;
        depth_found = true;
      } else if (module_name == "RGB Camera") {
        color_sensor = sensor;
        color_found = true;
      }
    }
    if (depth_found) {
      depth_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
      depth_sensor.set_option(RS2_OPTION_EXPOSURE, 10000);
      depth_sensor.set_option(RS2_OPTION_GAIN, 32);
      depth_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
    }
    if (color_found) {
      color_sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
      color_sensor.set_option(RS2_OPTION_EXPOSURE, 100);
      color_sensor.set_option(RS2_OPTION_GAIN, 64);
      color_sensor.set_option(RS2_OPTION_FRAMES_QUEUE_SIZE, 2);
    }

    rs2::pipeline pipe(rs_ctx);
    rs2::config cfg;
    cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, config_.camera.width, config_.camera.height, RS2_FORMAT_Z16,
                      config_.camera.fps);
    cfg.enable_stream(RS2_STREAM_COLOR, config_.camera.width, config_.camera.height, RS2_FORMAT_BGR8,
                      config_.camera.fps);
    rs_pipelines.push_back(pipe);

    auto selection = pipe.start(cfg);
    auto sensor = selection.get_device().first<rs2::depth_sensor>();
    auto depth_scale = sensor.get_depth_scale();
    depth_scales[serial] = depth_scale;
  }

  camera_loop_ = std::make_unique<EventLoop>();
  camera_loop_->PushCyclicTask(
      [=] {
        std::vector<rs2::frame> new_frames;
        for (auto&& pipe : rs_pipelines) {
          rs2::frameset fs;
          if (pipe.poll_for_frames(&fs)) {
            for (const rs2::frame& f : fs)
              new_frames.emplace_back(f);
          }
        }
        for (const auto& frame : new_frames) {
          std::string serial = rs2::sensor_from_frame(frame)->get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
          std::string name{};
          for (const auto& s : config_.camera.sensors) {
            if (s.serial == serial) {
              name = s.name;
              break;
            }
          }
          if (name.empty()) {
            continue;
          }
          if (depth_scales.find(serial) == depth_scales.end()) {
            continue;
          }
          double scale = depth_scales.at(serial);

          rs2_stream stream_type = frame.get_profile().stream_type();
          auto current_time = std::chrono::steady_clock::now();
          if (stream_type == RS2_STREAM_COLOR) {
            auto rgb = cv::Mat(cv::Size(config_.camera.width, config_.camera.height), CV_8UC3, (void*)frame.get_data(),
                               cv::Mat::AUTO_STEP);
            observation_buf_.PushTask([name, rgb = rgb.clone(), current_time, this] {
              obs_state.camera_updated_time = current_time;
              obs.images[name + "_rgb"] = rgb;
            });
          } else if (stream_type == RS2_STREAM_DEPTH) {
            auto depth = frame.as<rs2::depth_frame>();
            auto depth_image = cv::Mat(cv::Size(config_.camera.width, config_.camera.height), CV_16UC1,
                                       (void*)depth.get_data(), cv::Mat::AUTO_STEP);
            depth_image *= (scale * 1000);  // meter to milli-meter
            observation_buf_.PushTask([name, depth_image = depth_image.clone(), current_time, this] {
              obs_state.camera_updated_time = current_time;
              obs.images[name + "_depth"] = depth_image;
            });
          }
        }
      },
      std::chrono::milliseconds(1));
}

bool IntegratedRobot::IsReady() {
  if (robot_command_stream_handler_) {
    if (robot_command_stream_handler_->IsDone()) {
      return false;
    }
  } else {
    return false;
  }

  if (gripper_state_.load() != 4) {
    return false;
  }

  if (obs.images.size() != config_.camera.sensors.size() * 2 /* rgb, depth */) {
    return false;
  }

  return true;
}

bool IntegratedRobot::WaitUntilReady(std::chrono::milliseconds timeout) {
  using namespace std::chrono;
  auto start_time = steady_clock::now();
  while (steady_clock::now() - start_time < timeout) {
    if (IsReady()) {
      return true;
    }
    std::this_thread::sleep_for(milliseconds(1));
  }
  return false;
}

IntegratedRobot::Observation IntegratedRobot::GetObservation() {
  return observation_buf_.DoTask([=] { return obs; });
}

void IntegratedRobot::Step(const IntegratedRobot::Action& action, double minimum_time) {
  if (!IsReady()) {
    return;
  }

  Eigen::Vector<double, kRobotDOF + kGripperDOF> q_ref = action.actions;

  q_ref(2 + 6 + 7 + 7 + 0) = std::clamp(q_ref(2 + 6 + 7 + 7 + 0), -0.523, 0.523);
  q_ref(2 + 6 + 7 + 7 + 1) = std::clamp(q_ref(2 + 6 + 7 + 7 + 1), -0.35, 1.57);
  for (int i = 2; i < 2 + 6 + 7 + 7 + 2; i++) {
    q_ref(i) = std::clamp(q_ref(i), q_lower_limit_(i), q_upper_limit_(i));
  }

  auto torso_velocity_limit = Eigen::Vector<double, 6>::Constant(160. * M_PI / 180. * 0.9);
  auto torso_acc_limit = Eigen::Vector<double, 6>::Constant(600. * M_PI / 180. * 1.5);

  Eigen::Vector<double, 7> arm_velocity_limit;
  arm_velocity_limit << 160, 160, 160, 160, 330, 330, 330;
  arm_velocity_limit *= M_PI / 180.;
  arm_velocity_limit *= 0.9;
  auto arm_acc_limit = Eigen::Vector<double, 7>::Constant(1200. * M_PI / 180. * 1.5);

  auto head_velocity_limit = Eigen::Vector<double, 2>::Constant(1.);
  auto head_acc_limit = Eigen::Vector<double, 2>::Constant(3.);

  ComponentBasedCommandBuilder builder;
  if (config_.robot.act_mobility) {
    builder.SetMobilityCommand(JointVelocityCommandBuilder()
                                   .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.e9 /* Inf */))
                                   .SetVelocity(q_ref.block<2, 1>(0, 0))
                                   .SetAccelerationLimit(Eigen::Vector<double, 2>::Constant(0.1)));
  }
  builder
      .SetBodyCommand(
          BodyComponentBasedCommandBuilder()
              .SetTorsoCommand(JointPositionCommandBuilder()
                                   .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.e9 /* Inf */))
                                   .SetMinimumTime(minimum_time)
                                   .SetPosition(q_ref.block<6, 1>(2, 0))
                                   .SetVelocityLimit(torso_velocity_limit)
                                   .SetAccelerationLimit(torso_acc_limit))
              .SetRightArmCommand(JointPositionCommandBuilder()
                                      .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.e9 /* Inf */))
                                      .SetMinimumTime(minimum_time)
                                      .SetPosition(q_ref.block<7, 1>(2 + 6, 0))
                                      .SetVelocityLimit(arm_velocity_limit)
                                      .SetAccelerationLimit(arm_acc_limit))
              .SetLeftArmCommand(JointPositionCommandBuilder()
                                     .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.e9 /* Inf */))
                                     .SetMinimumTime(minimum_time)
                                     .SetPosition(q_ref.block<7, 1>(2 + 6 + 7, 0))
                                     .SetVelocityLimit(arm_velocity_limit)
                                     .SetAccelerationLimit(arm_acc_limit)))
      .SetHeadCommand(JointPositionCommandBuilder()
                          .SetCommandHeader(CommandHeaderBuilder().SetControlHoldTime(1.e9))
                          .SetMinimumTime(minimum_time)
                          .SetPosition(q_ref.block<2, 1>(2 + 6 + 7 + 7, 0))
                          .SetVelocityLimit(head_velocity_limit)
                          .SetAccelerationLimit(head_acc_limit));

  robot_command_stream_handler_->SendCommand(RobotCommandBuilder().SetCommand(builder));
  gripper_buf_.PushTask([=] {
    gripper_target_operation_mode.setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
    gripper_target_position = q_ref.block<2, 1>(2 + 6 + 7 + 7 + 2, 0);
  });
}

IntegratedRobot::Observation::Observation() = default;

IntegratedRobot::Observation::Observation(const Observation& other) {
  *this = other;
}

IntegratedRobot::Observation& IntegratedRobot::Observation::operator=(const Observation& other) {
  robot_qpos = other.robot_qpos;
  robot_qvel = other.robot_qvel;
  robot_torque = other.robot_torque;
  gripper_qpos = other.gripper_qpos;
  gripper_qvel = other.gripper_qvel;
  gripper_torque = other.gripper_torque;
  right_ft = other.right_ft;
  left_ft = other.left_ft;
  images.clear();
  for (const auto& [name, image] : other.images) {
    images[name] = image.clone();
  }
  robot_qpos_ref = other.robot_qpos_ref;
  return *this;
}

}  // namespace rb::y1a