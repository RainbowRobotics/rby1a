#pragma once

#include <string>

#include <rby1-sdk/base/dynamixel_bus.h>
#include <rby1-sdk/base/event_loop.h>
#include <rby1-sdk/model.h>
#include <rby1-sdk/robot.h>
#include <librealsense2/rs.hpp>
#include <librealsense2/rs_advanced_mode.hpp>
#include <opencv2/opencv.hpp>

namespace rb::y1a {

class IntegratedRobot {
 public:
  using Model = y1_model::A;
  static constexpr size_t kRobotDOF = Model::kRobotDOF;
  static constexpr size_t kGripperDOF = 2;

  struct ConfigRobot {
    std::string address{"192.168.30.1:50051"};

    bool act_mobility{false};

    bool obs_torque{true};
    bool obs_ft{true};
  };

  struct ConfigGripper {
    std::string dev_name{"/dev/rby1-gripper"};
  };

  struct ConfigCamera {
    int height{240};
    int width{424};
    int fps{60};

    struct Sensor {
      std::string name{};
      std::string serial{};
    };

    std::vector<Sensor> sensors;
  };

  struct Config {
    ConfigRobot robot;
    ConfigGripper gripper;
    ConfigCamera camera;
  };

  struct Observation {
    Observation();

    Observation(const Observation&);

    Observation& operator=(const Observation&);

    Eigen::Vector<double, kRobotDOF> robot_qpos;
    Eigen::Vector<double, kRobotDOF> robot_qvel;
    Eigen::Vector<double, kRobotDOF> robot_torque;  // if torque is enabled

    Eigen::Vector<double, kGripperDOF> gripper_qpos;
    Eigen::Vector<double, kGripperDOF> gripper_qvel;
    Eigen::Vector<double, kGripperDOF> gripper_torque;  // if torque is enabled

    Eigen::Vector<double, 6> right_ft;  // if ft is enabled
    Eigen::Vector<double, 6> left_ft;   // if ft is enabled

    std::unordered_map<std::string, cv::Mat> images;

    // TEMP
    Eigen::Vector<double, kRobotDOF> robot_qpos_ref;
  };

  struct ObservationState {
    std::chrono::steady_clock::time_point robot_updated_time;
    std::chrono::steady_clock::time_point gripper_updated_time;
    std::chrono::steady_clock::time_point camera_updated_time;
  };

  struct Action {
    Eigen::Vector<double, kRobotDOF + kGripperDOF> actions;
    // [0..2] mobility, [2..22] body, [22..24] head, [24..26] gripper
    // gripper >= 0 <= 1
  };

  static Config ParseConfig(const std::string& toml_str);

  explicit IntegratedRobot(const Config& config);

  explicit IntegratedRobot(const std::string& config_file);

  ~IntegratedRobot();

  Observation GetObservation();

  bool IsReady();

  bool WaitUntilReady(std::chrono::milliseconds timeout);

  void Step(const Action& action, double minimum_time = 1 /* (sec) */);

 private:
  void Initialize(const Config& config);
  void Initialize_robot();
  void Initialize_gripper();
  void Initialize_camera();

  rs2::context rs_ctx;

  Config config_;
  EventLoop observation_buf_;
  EventLoop gripper_buf_;

  Observation obs;
  ObservationState obs_state;

  std::shared_ptr<Robot<Model>> robot_;
  std::shared_ptr<rb::DynamixelBus> gripper_;

  std::shared_ptr<RobotCommandStreamHandler<Model>> robot_command_stream_handler_;

  std::atomic<bool> gripper_power_state_{false};
  std::unique_ptr<EventLoop> gripper_loop_;
  std::atomic<int> gripper_state_{0};
  Eigen::Vector<int, kGripperDOF> gripper_target_operation_mode{
      Eigen::Vector<int, kGripperDOF>::Constant(DynamixelBus::kCurrentControlMode)};
  Eigen::Vector<double, kGripperDOF> gripper_target_torque{Eigen::Vector<double, kGripperDOF>::Constant(0)};
  Eigen::Vector<double, kGripperDOF> gripper_target_position{Eigen::Vector<double, kGripperDOF>::Constant(0)};

  std::unique_ptr<EventLoop> camera_loop_;
};

}  // namespace rb::y1a