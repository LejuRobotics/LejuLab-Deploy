#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

using namespace leju;

class JointCmdSubscriber {
private:
    std::atomic<bool> running_;
    RobotBaseAPI* robot_api_;

public:
    JointCmdSubscriber() : running_(false), robot_api_(nullptr) {}

    ~JointCmdSubscriber() {
        stop();
    }

    bool initialize() {
        try {
            auto robot_version = leju::RobotVersion::from_env();
            GlobalRobot::init_env(robot_version);
            robot_api_ = &GlobalRobot::getInstance();

            robot_api_->subscribeRobotCmd([this](const RobotCmdConstPtr& msg) {
                std::cout << "Received robot command: " << *msg << std::endl;
            });

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize subscriber: " << e.what() << std::endl;
            return false;
        }
    }

    void run() {
        running_ = true;
        std::cout << "Joint command subscriber started." << std::endl;

        while (running_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        std::cout << "Joint command subscriber stopped." << std::endl;
    }

    void stop() {
        running_ = false;
    }

    bool isRunning() const {
        return running_;
    }
};

JointCmdSubscriber* g_subscriber = nullptr;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_subscriber) {
        g_subscriber->stop();
    }
}

int main(int argc, char* argv[]) {
    JointCmdSubscriber subscriber;
    g_subscriber = &subscriber;

    // Set up signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    if (!subscriber.initialize()) {
        std::cerr << "Failed to initialize subscriber" << std::endl;
        return -1;
    }

    std::cout << "Press Ctrl+C to exit." << std::endl;

    // Run the subscriber
    subscriber.run();

    std::cout << "Exiting..." << std::endl;
    return 0;
}