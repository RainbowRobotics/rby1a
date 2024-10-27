#include <iostream>

#include "rby1a/master_arm.h"

using namespace std::chrono_literals;
using namespace rb::y1a;

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config file>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};

  MasterArm master_arm(config_file);

  std::this_thread::sleep_for(10s);
}