#include "leju-rl-controller/joy_handler.h"

#include "leju-rl-controller/rl_log.h"

namespace leju {

JoyHandler::~JoyHandler() {
  stop();
}

void JoyHandler::setCallback(Callback cb) {
  callback_ = std::move(cb);
}

void JoyHandler::start() {
  if (running_) return;

  if (!GlobalRobot::is_initialized()) {
    RL_LOGE("JoyHandler: GlobalRobot not initialized");
    return;
  }

  running_ = true;

  auto& robot = GlobalRobot::getInstance();
  robot.subscribeJoyData([this](const JoyDataConstPtr& joy) {
    if (!running_ || !callback_) return;

    callback_(*joy, prev_buttons_);
    prev_buttons_ = joy->buttons;
  });

  RL_LOGI("JoyHandler started");
}

void JoyHandler::stop() {
  running_ = false;
  RL_LOGI("JoyHandler stopped");
}

}  // namespace leju
