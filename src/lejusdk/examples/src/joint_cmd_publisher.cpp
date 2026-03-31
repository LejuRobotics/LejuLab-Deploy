#include <iostream>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <cmath>
#include <vector>
#include <iomanip>
#include <mutex>

#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/robot_version.hpp"

using namespace leju;

class JointCmdPublisher {
private:
    std::atomic<bool> running_;
    RobotBaseAPI* robot_api_;
    size_t motor_count_;
    std::vector<std::string> motor_names_;
    std::atomic<bool> hardware_ready_;
    HardwareState current_hw_state_;
    std::mutex hw_state_mutex_;

public:
    JointCmdPublisher() : running_(false), robot_api_(nullptr), motor_count_(0),
                         hardware_ready_(false), current_hw_state_(HardwareState::UNKNOWN) {}

    ~JointCmdPublisher() {
        stop();
    }

    bool initialize() {
        try {
            auto robot_version = leju::RobotVersion::from_env();
            GlobalRobot::init_env(robot_version);
            robot_api_ = &GlobalRobot::getInstance();

            motor_count_ = robot_api_->getMotorNumber();
            motor_names_ = robot_api_->getMotorNames();

            std::cout << "Initialized with " << motor_count_ << " motors: ";
            for (size_t i = 0; i < motor_names_.size(); ++i) {
                if (i > 0) std::cout << ", ";
                std::cout << motor_names_[i];
            }
            std::cout << std::endl;

            return true;
        } catch (const std::exception& e) {
            std::cerr << "Failed to initialize publisher: " << e.what() << std::endl;
            return false;
        }
    }

    void hardwareStateCallback(const StringDataConstPtr& hw_state_data) {
        std::lock_guard<std::mutex> lock(hw_state_mutex_);
        std::string hw_state_str = hw_state_data->data;
        current_hw_state_ = String2HwState(hw_state_str);

        if (current_hw_state_ == HardwareState::READY_OK) {
            if (!hardware_ready_) {
                hardware_ready_ = true;
                std::cout << "Hardware is READY_OK, starting to publish commands..." << std::endl;
            }
        } else {
            if (hardware_ready_) {
                hardware_ready_ = false;
                std::cout << "Hardware state changed from READY_OK to: "
                         << HardwareState2String(current_hw_state_) << std::endl;
            }
        }
    }

    void waitForHardwareReady() {
        std::cout << "Waiting for hardware to be READY_OK..." << std::endl;
        auto start_time = std::chrono::steady_clock::now();

        while (!hardware_ready_ && running_) {
            auto current_time = std::chrono::steady_clock::now();
            auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(
                current_time - start_time).count();

            if (elapsed_seconds % 5 == 0 && elapsed_seconds > 0) {
                std::lock_guard<std::mutex> lock(hw_state_mutex_);
                std::cout << "Current hardware state: "
                         << HardwareState2String(current_hw_state_)
                         << " (waiting for READY_OK...)" << std::endl;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // Return when hardware is ready or running is false
    }

    RobotCmd generateSineWaveCommand(double time) {
        RobotCmd cmd(motor_count_);
        const double frequency = 0.5;
        const double amplitude = 0.2;

        size_t arm_head_base_index = 12;
        if(IS_KUAVO4PRO_LEGGED(robot_api_->getRobotVersion())) {
            arm_head_base_index = 12;
        }
        else if (IS_ROBAN_LEGGED(robot_api_->getRobotVersion()))
        {
            arm_head_base_index = 13;
        }
        else if(IS_KUAVO5_LEGGED(robot_api_->getRobotVersion())) {
            arm_head_base_index = 13;
        }
        
        size_t head_index = motor_count_ - 2;
        for (size_t i = arm_head_base_index; i < motor_count_; ++i) {
            cmd.q[i] = 0.0;
            // cmd.q[i] = amplitude * sin(1.0 * M_PI * frequency * time);
            cmd.v[i] = 0.0;
            cmd.tau[i] = 0.0;
            cmd.kp[i] = 20.0;
            cmd.kd[i] = 2.0;
            cmd.modes[i] = static_cast<uint8_t>(MotorControlMode::CSP);
        }

        // Only Head
        for (size_t i = head_index; i < motor_count_; ++i) {
            cmd.q[i] = amplitude * sin(1.0 * M_PI * frequency * time);
        }

        cmd.timestamp = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();

        return cmd;
    }

    void run() {
        running_ = true;
        std::cout << "Joint command publisher started." << std::endl;
        std::cout << "Subscribing to hardware state..." << std::endl;

        robot_api_->subscribeHardwareState(
            std::bind(&JointCmdPublisher::hardwareStateCallback, this, std::placeholders::_1)
        );

        std::cout << "Press Ctrl+C to stop." << std::endl;

        waitForHardwareReady();

        std::cout << "Publishing sine wave commands at 50Hz..." << std::endl;
        auto start_time = std::chrono::steady_clock::now();
        int iteration = 0;

        while (running_) {
            auto current_time = std::chrono::steady_clock::now();
            double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();

            RobotCmd cmd = generateSineWaveCommand(elapsed_seconds);

            bool success = robot_api_->publishRobotCmd(cmd);

            if (!success && iteration % 100 == 0) {
                std::cout << "Warning: Failed to publish command at iteration " << iteration << std::endl;
            }

            if (iteration % 100 == 0) {
                std::cout << cmd << std::endl;
            }

            iteration++;
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cout << "Joint command publisher stopped." << std::endl;
    }

    void stop() {
        running_ = false;
    }
};

JointCmdPublisher* g_publisher = nullptr;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_publisher) {
        g_publisher->stop();
    }
}

int main(int argc, char* argv[]) {
    std::cout << "=== Joint Command Publisher Example ===" << std::endl;
    std::cout << "This example publishes sine wave joint commands to control the robot." << std::endl;

    JointCmdPublisher publisher;
    g_publisher = &publisher;

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    if (!publisher.initialize()) {
        std::cerr << "Failed to initialize publisher" << std::endl;
        return -1;
    }

    publisher.run();

    std::cout << "Exiting..." << std::endl;
    return 0;
}