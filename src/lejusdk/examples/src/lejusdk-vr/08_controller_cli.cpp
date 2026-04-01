/**
 * @file 08_controller_cli.cpp
 * @brief Interactive Controller CLI Tool
 *
 * Based on lejusdk-vr interfaces, supports:
 * - list: List current controller state and available controllers
 * - switch <name>: Switch to specified controller
 * - mode <arm|waist> <keep|auto|external>: Set control mode
 * - help: Show help
 * - quit/exit: Exit the program
 */

#include <lejusdk-vr/lejusdk_vr.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <cstring>
#include <vector>

using namespace leju::vr;

// ANSI Color Codes
namespace color {
constexpr const char* RESET   = "\033[0m";
constexpr const char* BLACK   = "\033[30m";
constexpr const char* RED     = "\033[31m";
constexpr const char* GREEN   = "\033[32m";
constexpr const char* YELLOW  = "\033[33m";
constexpr const char* BLUE    = "\033[34m";
constexpr const char* MAGENTA = "\033[35m";
constexpr const char* CYAN    = "\033[36m";
constexpr const char* WHITE   = "\033[37m";
constexpr const char* BOLD    = "\033[1m";
constexpr const char* DIM     = "\033[2m";
}  // namespace color

// Convert control mode to readable string with color
std::string modeToString(ControlMode mode) {
  switch (mode) {
    case ControlMode::kKeepPose:
      return std::string(color::YELLOW) + "KeepPose" + color::RESET;
    case ControlMode::kAuto:
      return std::string(color::GREEN) + "Auto" + color::RESET;
    case ControlMode::kExternal:
      return std::string(color::CYAN) + "External" + color::RESET;
    default:
      return std::string(color::RED) + "Unknown" + color::RESET;
  }
}

// Convert string to control mode
bool stringToMode(const std::string& str, ControlMode& mode) {
  if (str == "keep" || str == "keeppose" || str == "0") {
    mode = ControlMode::kKeepPose;
    return true;
  } else if (str == "auto" || str == "1") {
    mode = ControlMode::kAuto;
    return true;
  } else if (str == "external" || str == "2") {
    mode = ControlMode::kExternal;
    return true;
  }
  return false;
}

// Print controller state with colors
void printControllerState(const ControllerState& state) {
  std::cout << "\n" << color::BOLD << "╔══════════════════════════════════════╗" << color::RESET << std::endl;
  std::cout << color::BOLD << "║        Controller State              ║" << color::RESET << std::endl;
  std::cout << color::BOLD << "╠══════════════════════════════════════╣" << color::RESET << std::endl;
  std::cout << "║ " << color::BOLD << "Current Controller:" << color::RESET << " " << color::GREEN << state.current_controller << color::RESET << std::endl;
  std::cout << "║ " << color::BOLD << "Arm Mode:  " << color::RESET << " " << modeToString(state.arm_mode) << std::endl;
  std::cout << "║ " << color::BOLD << "Waist Mode:" << color::RESET << " " << modeToString(state.waist_mode) << std::endl;
  std::cout << color::BOLD << "╠══════════════════════════════════════╣" << color::RESET << std::endl;
  std::cout << "║ " << color::BOLD << "Available Controllers:" << color::RESET << std::endl;
  for (size_t i = 0; i < state.available_controllers.size(); ++i) {
    const auto& ctrl = state.available_controllers[i];
    std::cout << "║   [" << color::CYAN << i << color::RESET << "] ";
    if (ctrl == state.current_controller) {
      std::cout << color::GREEN << ctrl << color::RESET << color::YELLOW << "  <-- current" << color::RESET;
    } else {
      std::cout << ctrl;
    }
    std::cout << std::endl;
  }
  std::cout << color::BOLD << "╚══════════════════════════════════════╝" << color::RESET << "\n" << std::endl;
}

// Refresh and show current state
bool refreshAndShow(VRBaseAPI& vr) {
  ControllerState state;
  if (!vr.getControllerState(state)) {
    std::cerr << color::RED << "Error: Failed to get controller state" << color::RESET << std::endl;
    return false;
  }
  printControllerState(state);
  return true;
}

// Split command line arguments
std::vector<std::string> splitArgs(const std::string& line) {
  std::vector<std::string> args;
  std::istringstream iss(line);
  std::string arg;
  while (iss >> arg) {
    args.push_back(arg);
  }
  return args;
}

// Show help information with colors
void showHelp() {
  std::cout << color::BOLD << "\nUsage:" << color::RESET << std::endl;
  std::cout << "  " << color::CYAN << "list" << color::RESET << "                    List current controller state and available controllers" << std::endl;
  std::cout << "  " << color::CYAN << "switch" << color::RESET << " " << color::YELLOW << "<name>" << color::RESET << "           Switch to specified controller" << std::endl;
  std::cout << "  " << color::CYAN << "mode" << color::RESET << " " << color::YELLOW << "<part> <mode>" << color::RESET << "      Set control mode" << std::endl;
  std::cout << "                          " << color::DIM << "part: arm | waist" << color::RESET << std::endl;
  std::cout << "                          " << color::DIM << "mode: keep | auto | external" << color::RESET << std::endl;
  std::cout << "  " << color::CYAN << "clear" << color::RESET << " / " << color::CYAN << "cls" << color::RESET << "              Clear the screen" << std::endl;
  std::cout << "  " << color::CYAN << "help" << color::RESET << "                    Show this help message" << std::endl;
  std::cout << "  " << color::CYAN << "quit" << color::RESET << " / " << color::CYAN << "exit" << color::RESET << "             Exit the program" << std::endl;

  std::cout << color::BOLD << "\nExamples:" << color::RESET << std::endl;
  std::cout << "  " << color::GREEN << "switch amp" << color::RESET << "              Switch to amp controller" << std::endl;
  std::cout << "  " << color::GREEN << "mode arm external" << color::RESET << "       Set arm to external control mode" << std::endl;
  std::cout << "  " << color::GREEN << "mode waist auto" << color::RESET << "         Set waist to auto mode" << std::endl;
  std::cout << std::endl;
}

// Print banner
void printBanner() {
  std::cout << color::BOLD << color::BLUE
            << "╔══════════════════════════════════════════╗\n"
            << "║                                          ║\n"
            << "║    Interactive Controller CLI Tool       ║\n"
            << "║                                          ║\n"
            << "╚══════════════════════════════════════════╝"
            << color::RESET << std::endl;
}

// Build prompt string
std::string buildPrompt(VRBaseAPI& /*vr*/) {
  std::string prompt = std::string(color::DIM) + "[" + color::GREEN + "leju-cli"
                     + color::DIM + "] " + color::RESET
                     + color::BOLD + color::BLUE + "> " + color::RESET;
  return prompt;
}

// Read line from stdin
std::string readline_with_prompt(const std::string& prompt) {
  std::cout << prompt;
  std::flush(std::cout);
  std::string line;
  if (!std::getline(std::cin, line)) {
    return "";
  }
  return line;
}

int main(int argc, char* argv[]) {
  printBanner();

  // Create and initialize API
  KuavoVRAPI vr;
  std::cout << "Initializing VR API..." << std::endl;
  if (!vr.initialize()) {
    std::cerr << color::RED << "Error: Initialization failed" << color::RESET << std::endl;
    return 1;
  }
  std::cout << color::GREEN << "Initialization successful!" << color::RESET << "\n" << std::endl;

  std::cout << "Type 'help' for available commands, 'quit' to exit.\n" << std::endl;

  // Main loop
  while (true) {
    std::string prompt = buildPrompt(vr);
    std::string line;

    line = readline_with_prompt(prompt);
    if (line.empty() && std::cin.eof()) {
      break;  // EOF
    }

    // Skip empty lines
    if (line.empty() || line.find_first_not_of(" \t") == std::string::npos) {
      continue;
    }

    auto args = splitArgs(line);
    if (args.empty()) {
      continue;
    }

    const std::string& cmd = args[0];

    if (cmd == "quit" || cmd == "exit" || cmd == "q") {
      std::cout << color::GREEN << "Goodbye!" << color::RESET << std::endl;
      break;
    } else if (cmd == "help" || cmd == "h" || cmd == "?") {
      showHelp();
    } else if (cmd == "list" || cmd == "ls") {
      refreshAndShow(vr);
    } else if (cmd == "clear" || cmd == "cls") {
      // Clear screen using ANSI escape codes
      std::cout << "\033[2J\033[H" << std::flush;
    } else if (cmd == "refresh" || cmd == "r") {
      refreshAndShow(vr);
    } else if (cmd == "switch" || cmd == "sw") {
      if (args.size() < 2) {
        std::cerr << color::RED << "Error: " << color::RESET << "Please specify controller name" << std::endl;
        std::cerr << "Usage: " << color::CYAN << "switch <controller_name>" << color::RESET << std::endl;
        continue;
      }
      const std::string& target = args[1];
      std::cout << "Switching to controller: " << color::YELLOW << target << color::RESET << "..." << std::endl;
      if (vr.switchController(target)) {
        std::cout << color::GREEN << "Switch successful!" << color::RESET << std::endl;
      } else {
        std::cerr << color::RED << "Error: Switch failed" << color::RESET << std::endl;
      }
    } else if (cmd == "mode") {
      if (args.size() < 3) {
        std::cerr << color::RED << "Error: " << color::RESET << "Insufficient arguments" << std::endl;
        std::cerr << "Usage: " << color::CYAN << "mode <arm|waist> <keep|auto|external>" << color::RESET << std::endl;
        continue;
      }

      const std::string& part = args[1];
      const std::string& modeStr = args[2];
      ControlMode mode;

      if (!stringToMode(modeStr, mode)) {
        std::cerr << color::RED << "Error: " << color::RESET << "Invalid mode '" << modeStr << "'" << std::endl;
        std::cerr << "Available modes: " << color::CYAN << "keep, auto, external" << color::RESET << std::endl;
        continue;
      }

      bool success = false;
      std::string partName;
      if (part == "arm") {
        success = vr.setArmMode(mode);
        partName = "Arm";
      } else if (part == "waist") {
        success = vr.setWaistMode(mode);
        partName = "Waist";
      } else {
        std::cerr << color::RED << "Error: " << color::RESET << "Invalid part '" << part << "'" << std::endl;
        std::cerr << "Available parts: " << color::CYAN << "arm, waist" << color::RESET << std::endl;
        continue;
      }

      if (success) {
        std::cout << color::GREEN << partName << " mode set to: " << modeToString(mode) << color::RESET << std::endl;
      } else {
        std::cerr << color::RED << "Error: Failed to set " << partName << " mode" << color::RESET << std::endl;
      }
    } else {
      std::cerr << color::RED << "Unknown command: '" << cmd << "'" << color::RESET << std::endl;
      std::cerr << "Type " << color::CYAN << "'help'" << color::RESET << " for available commands" << std::endl;
    }
  }

  // Shutdown
  vr.shutdown();
  return 0;
}
