#pragma once

#include <iostream>

#include <rby1a/integrated_robot.h>
#include <rby1a/master_arm.h>

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

  explicit AppMain(const std::string& config_file);

  ~AppMain();

  void Wait();

  void StartTeleoperation();

  void StopTeleoperation();

  void Zero();

  void Ready();

 private:
  void Initialize(const Config& config);
  void InitializeServer();

  Config config_;

  std::shared_ptr<rb::Robot<rb::y1_model::A>> robot_;
  std::unique_ptr<rb::y1a::MasterArm> master_;
  std::unique_ptr<rb::y1a::IntegratedRobot> slave_;

  zmq::context_t zmq_ctx_;
  zmq::socket_t srv_sock_;
  zmq::socket_t pub_sock_;

  rb::EventLoop state_buf_;
  rb::EventLoop publisher_ev_;
  rb::EventLoop service_ev_;

  State state_;
};