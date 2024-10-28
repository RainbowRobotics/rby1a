#include "rby1a/master_arm.h"

#include "toml++/toml.hpp"

namespace {
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

MasterArm::Config MasterArm::ParseConfig(const std::string& toml_str) {
  using namespace toml;

  Config config;
  auto tbl = toml::parse(toml_str);
  auto master_arm = tbl["master_arm"];

  config.robot_address = master_arm["robot_address"].value_or(config.robot_address);

  config.dev_name = master_arm["dev_name"].value_or(config.dev_name);
  config.model = master_arm["model"].value_or(config.model);

  const auto& get_values = [&](const std::string& key,
                               const Eigen::Vector<double, upc::MasterArm::kDOF>& default_value) {
    Eigen::Vector<double, upc::MasterArm::kDOF> v{default_value};
    if (auto arr = master_arm[key].as_array()) {
      size_t len = arr->size();
      if (len == config.q_min.size()) {
        for (int i = 0; i < len; i++) {
          v[i] = arr->at(i).value_or<double>(0.);
        }
      }
    }
    return v;
  };
  config.q_min = get_values("q_min", config.q_min);
  config.q_max = get_values("q_max", config.q_max);
  config.limit_barrier_gain = get_values("limit_barrier_gain", config.limit_barrier_gain);
  config.friction_viscous = get_values("friction_viscous", config.friction_viscous);

  return config;
}

MasterArm::MasterArm(const MasterArm::Config& config) {
  Initialize(config);
}

MasterArm::MasterArm(const std::string& config_file) : MasterArm(ParseConfig(ReadFileToString(config_file))) {}

MasterArm::~MasterArm() {
  std::cout << "destructo master arm" << std::endl;
  robot_->PowerOff("12v");
}

upc::MasterArm::State MasterArm::GetState() {
  return state_buf_.DoTask([=] { return state_; });
}

void MasterArm::Initialize(const MasterArm::Config& config) {
  config_ = config;

  /********************************
   * Robot
   ********************************/
  robot_ = Robot<Model>::Create(config_.robot_address);
  if (!robot_->Connect(1 /* iteration */, 1000 /* (ms) */)) {
    throw std::runtime_error("failed to connect robot");
  }
  // power on
  if (!robot_->IsPowerOn("12v")) {
    if (!robot_->PowerOn("12v")) {
      throw std::runtime_error("failed to power on");
    }
  }

  /********************************
   * Master Arm
   ********************************/
  try {
    // Latency timer setting
    upc::InitializeDevice(config_.dev_name);
  } catch (std::exception& e) {
    std::stringstream ss;
    ss << "Failed to initialize device: " << e.what();
    throw std::runtime_error(ss.str());
  }
  master_arm_ = std::make_shared<upc::MasterArm>(config_.dev_name);
  master_arm_->SetModelPath(config_.model);
  master_arm_->SetControlPeriod(0.04);

  auto active_ids = master_arm_->Initialize();
  if (active_ids.size() != upc::MasterArm::kDOF + 2) {
    throw std::runtime_error("failed to active master arm motors");
  }

  static bool ma_init = false;
  static Eigen::Vector<double, upc::MasterArm::kDOF / 2> ma_q_right, ma_q_left;
  ma_init = false;
  ma_q_right.setZero();
  ma_q_left.setZero();
  master_arm_->StartControl([=](const upc::MasterArm::State& state) {
    upc::MasterArm::ControlInput input;

    state_buf_.PushTask([=] { state_ = state; });

    if (!ma_init) {
      ma_q_right = state.q_joint(Eigen::seq(0, 6));
      ma_q_left = state.q_joint(Eigen::seq(7, 13));
      ma_init = true;
    }

    const auto& update_input = [=, &input](auto seq) {
      input.target_operation_mode(seq).setConstant(DynamixelBus::kCurrentControlMode);
      input.target_torque(seq) =
          (state.gravity_term(seq) +
           ((config_.q_min(seq) - state.q_joint(seq)).cwiseMax(0).array() * config_.limit_barrier_gain(seq).array() +
            (config_.q_max(seq) - state.q_joint(seq)).cwiseMin(0).array() * config_.limit_barrier_gain(seq).array() +
            state.qvel_joint(seq).array() * config_.friction_viscous(seq).array())
               .matrix())
              .cwiseMin(config_.torque_max(seq))
              .cwiseMax(-config_.torque_max(seq));
    };

    if (state.button_right.button == 1) {
      update_input(Eigen::seq(0, 6));
      ma_q_right = state.q_joint(Eigen::seq(0, 6));
    } else {
      input.target_operation_mode(Eigen::seq(0, 6)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(0, 6)) = ma_q_right;
    }

    if (state.button_left.button == 1) {
      update_input(Eigen::seq(7, 13));
      ma_q_left = state.q_joint(Eigen::seq(7, 13));
    } else {
      input.target_operation_mode(Eigen::seq(7, 13)).setConstant(DynamixelBus::kCurrentBasedPositionControlMode);
      input.target_position(Eigen::seq(7, 13)) = ma_q_left;
    }

    return input;
  });
}

}  // namespace rb::y1a