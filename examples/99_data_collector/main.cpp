#include <iostream>

#include "app_main.h"

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config file>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};

  AppMain app(config_file);

  return 0;
}