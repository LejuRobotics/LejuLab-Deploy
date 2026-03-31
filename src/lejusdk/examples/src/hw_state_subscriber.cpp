#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

using namespace leju;

class HardwareStateSubscriber {
private:
    std::atomic<bool> running_;
    RobotBaseAPI* robot_api_;

public:
    HardwareStateSubscriber() : running_(false), robot_api_(nullptr) {}

    ~HardwareStateSubscriber() {
        stop();
    }

    bool initialize() {
        try {
            auto robot_version = leju::RobotVersion::from_env();
            GlobalRobot::init_env(robot_version);
            robot_api_ = &GlobalRobot::getInstance();

            robot_api_->subscribeHardwareState([this](const StringDataConstPtr& msg) {
                // Convert string to enum for better understanding
                HardwareState state = String2HwState(msg->data);
                std::cout << "Hardware state : " << HardwareState2String(state) << std::endl;
            });

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize hardware state subscriber: " << e.what() << std::endl;
            return false;
        }
    }

    void run() {
        running_ = true;
        std::cout << "Hardware state subscriber started." << std::endl;
        std::cout << "Waiting for hardware state updates..." << std::endl;

        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Hardware state subscriber stopped." << std::endl;
    }

    void stop() {
        running_ = false;
    }

    bool isRunning() const {
        return running_;
    }
};

HardwareStateSubscriber* g_subscriber = nullptr;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_subscriber) {
        g_subscriber->stop();
    }
}

int main(int argc, char* argv[]) {
    HardwareStateSubscriber subscriber;
    g_subscriber = &subscriber;

    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    if (!subscriber.initialize()) {
        std::cerr << "Failed to initialize hardware state subscriber" << std::endl;
        return -1;
    }

    std::cout << "Press Ctrl+C to exit." << std::endl;

    // Run the subscriber
    subscriber.run();

    std::cout << "Exiting..." << std::endl;
    return 0;
}