#pragma once

#include <string>

#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>
#include <rby1-sdk/upc/master_arm.h>

namespace rb::y1a {

class MasterArm {
 public:
  using Model = y1_model::A;

  struct Config {
    std::string robot_address{"192.168.30.1:50051"};

    std::string dev_name{"/dev/rby1_master_arm"};
    std::string model{"/usr/local/share/rby1-sdk/models/master_arm/model.urdf"};

    Eigen::Vector<double, upc::MasterArm::kDOF> torque_max{Eigen::Vector<double, upc::MasterArm::kDOF>::Constant(3.)};
    Eigen::Vector<double, upc::MasterArm::kDOF> q_min{-6.28318531, -0.52359878, 0.0,         -2.35619449, -1.57079633,
                                                      0.61086524,  -6.28318531, -6.28318531, 0.17453293,  -1.57079633,
                                                      -2.35619449, -1.57079633, 0.61086524,  -6.28318531};
    Eigen::Vector<double, upc::MasterArm::kDOF> q_max{6.28318531,  -0.17453293, 1.57079633, -1.04719755, 1.57079633,
                                                      1.3962634,   6.28318531,  6.28318531, 0.52359878,  0.0,
                                                      -1.04719755, 1.57079633,  1.3962634,  6.28318531};
    Eigen::Vector<double, upc::MasterArm::kDOF> limit_barrier_gain{0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5,
                                                                   0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    Eigen::Vector<double, upc::MasterArm::kDOF> friction_viscous{0.01, 0.01, 0.01, 0.01, 0.005, 0.005, 0.001,
                                                                 0.01, 0.01, 0.01, 0.01, 0.005, 0.005, 0.001};
  };

  static Config ParseConfig(const std::string& toml_str);

  explicit MasterArm(const Config& config);

  explicit MasterArm(const std::string& config_file);

  ~MasterArm();

  upc::MasterArm::State GetState();

 private:
  void Initialize(const Config& config);

  Config config_;
  EventLoop state_buf_;

  upc::MasterArm::State state_;

  std::shared_ptr<Robot<Model>> robot_{nullptr};
  std::shared_ptr<rb::upc::MasterArm> master_arm_{nullptr};

  bool ma_init;
  Eigen::Vector<double, upc::MasterArm::kDOF / 2> ma_q_right, ma_q_left;
};

}  // namespace rb::y1a