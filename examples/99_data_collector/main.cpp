#include <iostream>
#include <csignal>

#include "app_main.h"

std::unique_ptr<AppMain> app;

void signal_handler(int signum) {
  std::cout << "signal handler" << std::endl;

  if (app) {
    app.reset();
  }

  _exit(EXIT_FAILURE);
  _exit(128 + signum);
  signal(SIGTERM, SIG_DFL);
  raise(SIGTERM);
}

int main(int argc, char** argv) {
  std::signal(SIGINT, signal_handler);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <config file>" << std::endl;
    return 1;
  }
  std::string config_file{argv[1]};

  app = std::make_unique<AppMain>(config_file);

  while (true) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  std::cout << "Finished" << std::endl;

  return 0;
}