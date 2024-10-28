#include "app_main.h"

#include <filesystem>
#include <string>

#include "highfive/H5Easy.hpp"
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

template <typename T>
void AddDataIntoDataSet(std::unique_ptr<HighFive::DataSet>& dataset, const std::vector<std::size_t>& shape, T* data) {
  auto current_dims = dataset->getSpace().getDimensions();
  size_t current_rows = current_dims[0];
  size_t new_rows = current_rows + 1;
  std::vector<std::size_t> resize = {new_rows}, offset = {current_rows}, count = {1};
  for (const auto& l : shape) {
    resize.push_back(l);
    offset.push_back(0);
    count.push_back(l);
  }
  dataset->resize(resize);
  dataset->select(offset, count).write_raw(data);
}

template <typename T>
HighFive::DataSet CreateDataSet(HighFive::File& file, const std::string& name, const std::vector<std::size_t>& shape,
                                int compression_level = 0) {
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
}  // namespace

AppMain::AppMain(const std::string& config_file) {
  using namespace toml;

  Config config;
  auto tbl = toml::parse_file(config_file);

  std::stringstream master_ss;
  master_ss << tbl["master"];
  config.master_config = rb::y1a::MasterArm::ParseConfig(master_ss.str());

  std::stringstream slave_ss;
  slave_ss << tbl["slave"];
  config.slave_config = rb::y1a::IntegratedRobot::ParseConfig(slave_ss.str());

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
  robot_.reset();

  StopRecording().get();

  std::cout << "publisher_ev reset ... ";
  publisher_ev_->Stop();
  publisher_ev_->PurgeTasks();
  publisher_ev_.reset();
  std::cout << " - finished" << std::endl;

  std::cout << "service_ev reset ... ";
  service_ev_->Stop();
  service_ev_->PurgeTasks();
  service_ev_.reset();
  std::cout << " - finished" << std::endl;

  std::cout << "record_ev reset ... ";
  record_ev_.reset();
  std::cout << " - finished" << std::endl;

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  if (slave_) {
    slave_.reset();
    std::cout << "slave deleted" << std::endl;
  }

  if (master_) {
    master_.reset();
    std::cout << "master deleted" << std::endl;
  }

  std::cout << "state_buf reset ... ";
  state_buf_.reset();
  std::cout << " - finished" << std::endl;
}

void AppMain::Initialize(const Config& config) {
  using namespace rb;
  using namespace std::chrono_literals;

  config_ = config;

  publisher_ev_ = std::make_unique<rb::EventLoop>();
  state_buf_ = std::make_unique<rb::EventLoop>();
  service_ev_ = std::make_unique<rb::EventLoop>();
  record_ev_ = std::make_unique<rb::EventLoop>();

  //

  master_ = std::make_unique<y1a::MasterArm>(config_.master_config);

  std::cout << "Master Arm Initialized" << std::endl;

  slave_ = std::make_unique<y1a::IntegratedRobot>(config_.slave_config);
  if (!slave_->WaitUntilReady(15s)) {
    std::cout << "error" << std::endl;
    slave_.reset();
  }

  std::cout << "Integrated Robot Initialized" << std::endl;

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
        if (!state_buf_) {
          return;
        }

        state_buf_->PushTask([=] {
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

  publisher_ev_->PushCyclicTask(
      [=] {
        if (!state_buf_) {
          return;
        }

        using namespace nlohmann;

        state_buf_->PushTask([=] {
          std::filesystem::space_info si = std::filesystem::space(config_.record.path);
          state_.upc_storage_free = (double)si.free / (1 << 20);  // MB 단위로 변환
          state_.upc_storage_available = (double)si.available / (1 << 20);
          state_.upc_storage_capacity = (double)si.capacity / (1 << 20);

          if (master_) {
            state_.master_state = master_->GetState();
          }
          if (slave_) {
            state_.slave_observation = slave_->GetObservation();
          }
        });

        auto state = state_buf_->DoTask([=] { return state_; });

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
          static std::future<void> image_future;
          bool done = true;
          if (image_future.valid()) {
            if (image_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::timeout) {
              done = false;
            }
          }
          if (done) {
            image_future = std::async(std::launch::async, [s = state, this] {
              nlohmann::json image_j;
              for (const auto& [name, image] : s.slave_observation.images) {
                cv::Mat resizedImg;
                cv::resize(image, resizedImg, cv::Size(), 0.5, 0.5, cv::INTER_LINEAR);
                image_j[name] = MatToJson(resizedImg);
              }
              publisher_ev_->PushTask([=] {
                zmq::send_multipart(
                    pub_sock_, std::array<zmq::const_buffer, 2>{zmq::str_buffer("image"), zmq::buffer(image_j.dump())});
              });
            });
            last_time = std::chrono::steady_clock::now();
          }
        }
      },
      100ms);

  /************************
   * SERVICE
   ************************/

  service_ev_->PushCyclicTask(
      [=] {
        if (!state_buf_) {
          return;
        }

        State state = state_buf_->DoTask([=] { return state_; });

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
              state_buf_->Pause();
              state_buf_->WaitForTasks();
              master_ = std::make_unique<y1a::MasterArm>(config_.master_config);
              state_buf_->Unpause();
            }
          } else if (command == kCommandResetSlave) {
            if (!state.teleop) {
              state_buf_->Pause();
              state_buf_->WaitForTasks();
              slave_ = std::make_unique<y1a::IntegratedRobot>(config_.slave_config);
              if (!slave_->WaitUntilReady(10s)) {
                std::cout << "error" << std::endl;
                slave_.reset();
              }
              state_buf_->Unpause();
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
          } else if (command == kCommandStartRecording) {
            std::string name{"example"};
            if (j.contains("name")) {
              name = j["name"];
            }

            if (teleop_) {
              StartRecording(config_.record.path + "/" + name + ".h5");
            }
          } else if (command == kCommandStopRecording) {
            bool valid = true;
            if (j.contains("valid")) {
              valid = j["valid"];
            }

            StopRecording(valid);
          }
        } catch (...) {}
      },
      10ms);
}

void AppMain::Wait() {
  // TODO .. ?
  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

AppMain::Teleop::Teleop(AppMain* app) : app_(app) {
  app_->state_.teleop = true;

  std::cout << "Construct tele-operation" << std::endl;

  auto slave_obs = app_->slave_->GetObservation();
  qpos_ref_ = slave_obs.robot_qpos;

  auto rt_thread = std::make_unique<rb::Thread>();
  rt_thread->SetOSPriority(90, SCHED_RR);
  loop_ = std::make_unique<rb::EventLoop>(std::move(rt_thread));
  loop_->PushCyclicTask([=] { loop(); }, std::chrono::nanoseconds((long)(1.e9 / app_->config_.record.fps)));
}

AppMain::Teleop::~Teleop() {
  app_->StopRecording().get();
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
  yaw = std::clamp(yaw, -0.523, 0.523);
  pitch = std::clamp(pitch, -0.35, 1.57);
  qpos_ref_.block<2, 1>(2 + 6 + 7 + 7, 0) = Eigen::Vector<double, 2>{yaw, pitch};
  for (int i = 0; i < 6 + 14 + 2; i++) {
    qpos_ref_(2 + i) = std::clamp(qpos_ref_(2 + i), app_->q_lower_limit_(2 + i), app_->q_upper_limit_(2 + i));
  }

  rb::y1a::IntegratedRobot::Action action;
  action.actions.head<rb::y1a::IntegratedRobot::kRobotDOF>() = qpos_ref_;
  action.actions.tail<rb::y1a::IntegratedRobot::kGripperDOF>() =
      Eigen::Vector2d::Constant(1.) -
      Eigen::Vector2d{master_state.button_right.trigger, master_state.button_left.trigger} / 1000.;
  app_->slave_->Step(action, 1 / app_->config_.record.fps * 1.05);

  if (app_->state_.recording) {
    app_->Record(slave_obs, action);
    app_->record_data_count_++;
    if (app_->record_data_count_ % 100 == 0) {
      std::cout << app_->state_.recording_count << " / " << app_->record_data_count_ << std::endl;
    }
  }
}

void AppMain::StartRecording(const std::string& file_path) {
  if (!recording_ready_.load()) {
    return;
  }

  state_.recording = true;
  state_.recording_count = 0;

  record_ev_->PushTask([=] {
    if (record_file_) {
      return;
    }

    const int kCompressionLevel = 0;

    record_file_ = std::make_unique<HighFive::File>(file_path, HighFive::File::Overwrite);
    record_file_->createAttribute("frequency", config_.record.fps);

    // Create Dataset
    std::size_t height = config_.slave_config.camera.height;
    std::size_t width = config_.slave_config.camera.width;
    for (const auto& [name, _] : config_.slave_config.camera.sensors) {
      auto depth = std::make_unique<HighFive::DataSet>(CreateDataSet<uint16_t>(
          *record_file_, "/observations/images/" + name + "_depth", {height, width}, kCompressionLevel));
      depth->createAttribute("scale", 1000);
      record_depth_datasets_[name + "_depth"] = std::move(depth);

      record_rgb_datasets_[name + "_rgb"] = std::make_unique<HighFive::DataSet>(CreateDataSet<std::uint8_t>(
          *record_file_, "/observations/images/" + name + "_rgb", {height, width, 3}, kCompressionLevel));
    }
    record_action_dataset_ = std::make_unique<HighFive::DataSet>(CreateDataSet<double>(
        *record_file_, "/action", {rb::y1a::IntegratedRobot::kRobotDOF + rb::y1a::IntegratedRobot::kGripperDOF}));
    record_qpos_dataset_ = std::make_unique<HighFive::DataSet>(
        CreateDataSet<double>(*record_file_, "/observations/qpos",
                              {rb::y1a::IntegratedRobot::kRobotDOF + rb::y1a::IntegratedRobot::kGripperDOF}));
    record_qvel_dataset_ = std::make_unique<HighFive::DataSet>(
        CreateDataSet<double>(*record_file_, "/observations/qvel",
                              {rb::y1a::IntegratedRobot::kRobotDOF + rb::y1a::IntegratedRobot::kGripperDOF}));
    record_torque_dataset_ = std::make_unique<HighFive::DataSet>(
        CreateDataSet<double>(*record_file_, "/observations/torque",
                              {rb::y1a::IntegratedRobot::kRobotDOF + rb::y1a::IntegratedRobot::kGripperDOF}));
    record_ft_dataset_ =
        std::make_unique<HighFive::DataSet>(CreateDataSet<double>(*record_file_, "/observations/ft_sensor", {6 * 2}));

    record_data_count_ = 0;
    recording_ready_ = false;
  });
}

std::future<void> AppMain::StopRecording(bool valid) {
  if (recording_ready_.load()) {
    std::promise<void> f;
    f.set_value();
    return f.get_future();
  }

  state_.recording = false;

  return record_ev_->Submit([=] {
    if (!record_file_) {
      return;
    }

    record_file_->createAttribute("valid", valid);

    record_depth_datasets_.clear();
    record_rgb_datasets_.clear();
    record_action_dataset_.reset();
    record_qpos_dataset_.reset();
    record_qvel_dataset_.reset();
    record_torque_dataset_.reset();
    record_ft_dataset_.reset();
    record_file_->flush();
    record_file_.reset();
    record_file_ = nullptr;

    recording_ready_ = true;
  });
}

void AppMain::Record(const rb::y1a::IntegratedRobot::Observation& observation,
                     const rb::y1a::IntegratedRobot::Action& action) {
  using namespace rb::y1a;

  record_ev_->PushTask([obs = observation, act = action, this] {
    if (!state_.recording) {
      return;
    }

    if (!record_file_) {
      return;
    }

    std::size_t height = config_.slave_config.camera.height;
    std::size_t width = config_.slave_config.camera.width;
    Eigen::Vector<double, IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF> record_action;
    Eigen::Vector<double, IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF> record_qpos;
    Eigen::Vector<double, IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF> record_qvel;
    Eigen::Vector<double, IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF> record_torque;
    Eigen::Vector<double, 6 * 2> record_ft;
    record_action = act.actions;
    record_qpos.head<IntegratedRobot::kRobotDOF>() = obs.robot_qpos;
    record_qpos.tail<IntegratedRobot::kGripperDOF>() = obs.gripper_qpos;
    record_qvel.head<IntegratedRobot::kRobotDOF>() = obs.robot_qvel;
    record_qvel.tail<IntegratedRobot::kGripperDOF>() = obs.gripper_qvel;
    record_torque.head<IntegratedRobot::kRobotDOF>() = obs.robot_torque;
    record_torque.tail<IntegratedRobot::kGripperDOF>() = obs.gripper_torque;
    record_ft.head<6>() = obs.right_ft;
    record_ft.tail<6>() = obs.left_ft;

    for (const auto& [name, image] : obs.images) {
      if (image.type() == CV_16UC1) {
        if (record_depth_datasets_.find(name) == record_depth_datasets_.end()) {
          std::cerr << "Depth dataset (" << name << ") is not initialized" << std::endl;
          continue;
        }
        AddDataIntoDataSet(record_depth_datasets_[name], {height, width}, (uint16_t*)image.data);
      } else if (image.type() == CV_8UC3) {
        if (record_rgb_datasets_.find(name) == record_rgb_datasets_.end()) {
          std::cerr << "RGB dataset (" << name << ") is not initialized" << std::endl;
          continue;
        }
        AddDataIntoDataSet(record_rgb_datasets_[name], {height, width, 3}, image.data);
      }
    }
    AddDataIntoDataSet(record_action_dataset_, {IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF},
                       record_action.data());
    AddDataIntoDataSet(record_qpos_dataset_, {IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF},
                       record_qpos.data());
    AddDataIntoDataSet(record_qvel_dataset_, {IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF},
                       record_qvel.data());
    AddDataIntoDataSet(record_torque_dataset_, {IntegratedRobot::kRobotDOF + IntegratedRobot::kGripperDOF},
                       record_torque.data());
    AddDataIntoDataSet(record_ft_dataset_, {6 * 2}, record_ft.data());

    state_.recording_count++;
  });
}