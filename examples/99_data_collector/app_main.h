// Add this at the top of your source file
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#pragma once

#include <iostream>

#include <rby1a/integrated_robot.h>
#include <rby1a/master_arm.h>

#include "highfive/H5Easy.hpp"
#include "toml++/toml.hpp"
#include "zmq.hpp"

class AppMain {
 public:
  const std::string kCommandResetMaster = "reset_master";
  const std::string kCommandResetSlave = "reset_slave";
  const std::string kCommandZeroPose = "zero_pose";
  const std::string kCommandReadyPose = "ready_pose";
  const std::string kCommandStartTeleop = "start_teleop";
  const std::string kCommandStopTeleop = "stop_teleop";

  struct ConfigRecord {
    double fps{30};
    std::string path{"./"};
  };

  struct ConfigServer {
    std::string robot_address{"192.168.30.1:50051"};
    int request_server_port{5454};
    int publisher_server_port{5455};
  };

  struct Config {
    std::string master_config;  // master arm
    std::string slave_config;   // robot

    ConfigRecord record;
    ConfigServer server;
  };

  struct State {
    rb::upc::MasterArm::State master_state;
    rb::y1a::IntegratedRobot::Observation slave_observation;

    bool power_12v{false};
    bool power_48v{false};
    bool servo_on{false};
    bool control_manger{false};

    bool teleop{false};

    bool recording{false};
    int recording_count{0};

    double upc_storage_free{0.};
    double upc_storage_available{0.};
    double upc_storage_capacity{0.};
  };

  class Teleop {
   public:
    Teleop(AppMain* app);

    ~Teleop();

    void loop();

   private:
    AppMain* app_;

    rb::EventLoop loop_;
    Eigen::Vector<double, rb::y1_model::A::kRobotDOF> qpos_ref_;
    double right_ratio_{0};
    double left_ratio_{0};
  };

  explicit AppMain(const std::string& config_file);

  ~AppMain();

  void Wait();

  void StartTeleoperation();

  void StopTeleoperation();

  void Zero();

  void Ready();

  void StartRecord(const std::string& file);

  void StopRecord();

  void Record(rb::y1a::IntegratedRobot::Observation observation, rb::y1a::IntegratedRobot::Action action);

 private:
  void Initialize(const Config& config);

  void InitializeServer();

  Config config_;

  std::unique_ptr<Teleop> teleop_;

  std::shared_ptr<rb::Robot<rb::y1_model::A>> robot_;
  std::unique_ptr<rb::y1a::MasterArm> master_;
  std::unique_ptr<rb::y1a::IntegratedRobot> slave_;

  std::shared_ptr<rb::dyn::Robot<rb::y1_model::A::kRobotDOF>> robot_dyn_;
  std::shared_ptr<rb::dyn::State<rb::y1_model::A::kRobotDOF>> robot_dyn_state_;
  Eigen::Vector<double, rb::y1_model::A::kRobotDOF> q_lower_limit_, q_upper_limit_;

  zmq::context_t zmq_ctx_;
  zmq::socket_t srv_sock_;
  zmq::socket_t pub_sock_;

  rb::EventLoop state_buf_;
  rb::EventLoop publisher_ev_;
  rb::EventLoop service_ev_;
  rb::EventLoop record_ev_;

  State state_;

  std::unique_ptr<HighFive::File> record_file_{nullptr};
  std::vector<std::unique_ptr<HighFive::DataSet>> record_depth_datasets_;
  std::vector<std::unique_ptr<HighFive::DataSet>> record_rgb_datasets_;
  std::unique_ptr<HighFive::DataSet> record_action_dataset_;
  std::unique_ptr<HighFive::DataSet> record_qpos_dataset_;
  std::unique_ptr<HighFive::DataSet> record_qvel_dataset_;
  std::unique_ptr<HighFive::DataSet> record_torque_dataset_;
  std::unique_ptr<HighFive::DataSet> record_ft_dataset_;

  friend class Teleop;
};