#include "leju-dummy-controller/dummy_controller.h"
#include "leju-dummy-controller/joint_monkey_controller.h"
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

int main() {
  try {
    JointMonkeyController controller(leju::RobotVersion::from_env());
    if (!controller.initialize()) {
      std::cerr << "❌ Failed to initialize Kuavo robot controller"
                << std::endl;
      return 1;
    }
    controller.start();
    controller.mainLoop();
  } catch (const std::exception& e) {
    std::cerr << "❌ Kuavo example failed with exception: " << e.what()
              << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "❌ Kuavo example failed with unknown exception" << std::endl;
    return 1;
  }

  return 0;
}
