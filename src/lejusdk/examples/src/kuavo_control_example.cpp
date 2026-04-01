#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <atomic>
#include <mutex>
#include <vector>
#include <iomanip>
#include "lejusdk-lowlevel/leju_sdk.h"
#include "lejusdk-utils/time_utils.hpp"

using namespace leju;

/**
 * @brief Kuavo机器人控制类
 *
 * 封装Kuavo机器人的控制功能，包括数据订阅、正弦波控制和状态管理
 */
class KuavoRobotController {
public:
    /**
     * @brief 构造函数
     * @param version 机器人版本
     */
    explicit KuavoRobotController(const RobotVersion& version = RobotVersions::KUAVO5_BASE);

    /**
     * @brief 析构函数
     */
    ~KuavoRobotController();

    /**
     * @brief 初始化机器人
     * @return 初始化是否成功
     */
    bool initialize();

    /**
     * @brief 运行控制循环
     * @param duration 运行时长（秒）
     * @param control_frequency 控制频率（Hz）
     */
    void runControlLoop(double duration = 30.0, double control_frequency = 50.0);

    
private:
    // 机器人配置
    RobotVersion robot_version_;
    size_t motor_count_;
    std::vector<std::string> motor_names_;

    // 数据计数和同步
    std::atomic<int> imu_count_{0};
    std::atomic<int> joint_state_count_{0};
    mutable std::mutex print_mutex_;
    std::mutex robot_state_mutex_;

    // 保存最新的机器人状态
    RobotState latest_robot_state_;

  
    
    /**
     * @brief IMU数据订阅回调
     */
    void imuCallback(const ImuDataConstPtr& imu_data);

    /**
     * @brief 关节状态订阅回调
     */
    void jointStateCallback(const RobotStateConstPtr& robot_state);

    /**
     * @brief 使用正弦波生成关节控制指令
     * @param time 时间（秒）
     * @return 生成的控制指令
     */
    RobotCmd generateSineWaveCommand(double time) const;

    /**
     * @brief 获取关节振幅
     * @param joint_name 关节名称
     * @return 振幅值（弧度）
     */
    double getJointAmplitude(const std::string& joint_name) const;

    /**
     * @brief 设置数据订阅
     */
    void setupSubscriptions();

    /**
     * @brief 打印机器人信息
     */
    void printRobotInfo() const;
};

KuavoRobotController::KuavoRobotController(const RobotVersion& version)
    : robot_version_(version), motor_count_(0) {
}

KuavoRobotController::~KuavoRobotController() = default;

bool KuavoRobotController::initialize() {
    std::cout << "--- Initializing Kuavo Robot ---" << std::endl;

    // 初始化机器人
    if (!GlobalRobot::init_env(robot_version_)) {
        std::cerr << "❌ Failed to initialize Kuavo robot" << std::endl;
        return false;
    }

    // 获取机器人信息
    auto& robot = GlobalRobot::getInstance();
    motor_count_ = robot.getMotorNumber();
    motor_names_ = robot.getMotorNames();

    printRobotInfo();
    setupSubscriptions();

    std::cout << "✓ Kuavo robot initialized successfully" << std::endl;
    return true;
}

void KuavoRobotController::printRobotInfo() const {
    std::cout << "  Motor Count: " << motor_count_ << std::endl;
    std::cout << "  Motors: ";
    for (size_t i = 0; i < motor_names_.size(); ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << motor_names_[i];
    }
    std::cout << std::endl;
}

void KuavoRobotController::setupSubscriptions() {
    std::cout << "--- Setting Up Data Subscriptions ---" << std::endl;

    auto& robot = GlobalRobot::getInstance();
    robot.subscribeImuData([this](const ImuDataConstPtr& imu_data) {
        this->imuCallback(imu_data);
    });

    robot.subscribeRobotState([this](const RobotStateConstPtr& robot_state) {
        this->jointStateCallback(robot_state);
    });

    std::cout << "✓ Subscribed to IMU and Joint State topics" << std::endl;
}

void KuavoRobotController::imuCallback(const ImuDataConstPtr& imu_data) {
    int count = ++imu_count_;
    if (count % 10 == 1) {  // 每10条打印一次（第1, 11, 21, ...条）
        std::lock_guard<std::mutex> lock(print_mutex_);
        std::cout << "\n=== IMU Data [" << count << "] ===" << std::endl;
        // 使用重载的<<运算符直接输出
        std::cout << *imu_data << std::endl;
    }
}

void KuavoRobotController::jointStateCallback(const RobotStateConstPtr& robot_state) {
    int count = ++joint_state_count_;
    if (count % 10 == 1) {  // 每10条打印一次（第1, 11, 21, ...条）
        std::lock_guard<std::mutex> lock(print_mutex_);
        std::lock_guard<std::mutex> state_lock(robot_state_mutex_);

        // 保存最新的机器人状态
        latest_robot_state_ = *robot_state;

        std::cout << "\n=== Joint State Data [" << count << "] ===" << std::endl;
        // 使用重载的<<运算符直接输出
        std::cout << *robot_state << std::endl;
    }
}

double KuavoRobotController::getJointAmplitude(const std::string& joint_name) const {
    return 0.2;  // 所有关节使用统一振幅
}

RobotCmd KuavoRobotController::generateSineWaveCommand(double time) const {
    RobotCmd cmd(motor_count_);
    const double frequency = 0.5;  // 基础频率 0.5 Hz

    for (size_t i = 0; i < motor_count_ && i < motor_names_.size(); ++i) {
        const std::string& joint_name = motor_names_[i];
        double amplitude = getJointAmplitude(joint_name);

        // 生成正弦波位置指令
        cmd.q[i] = amplitude * sin(2.0 * M_PI * frequency * time);

        // 计算速度（正弦波的导数）
        cmd.v[i] = amplitude * 2.0 * M_PI * frequency * cos(2.0 * M_PI * frequency * time);

        // 力矩设为0（位置控制模式）
        cmd.tau[i] = 0.0;

        // 设置控制增益
        cmd.kp[i] = 20.0;  // 位置增益
        cmd.kd[i] = 2.0;   // 速度增益

        // 控制模式：CSP (位置控制)
        cmd.modes[i] = static_cast<uint8_t>(MotorControlMode::CSP);
    }

    // 设置时间戳（使用Unix时间戳，与ROS兼容）
    cmd.timestamp = leju::common::GetUnixTimestampS();

    return cmd;
}

void KuavoRobotController::runControlLoop(double duration, double control_frequency) {
    std::cout << "\n--- Starting Control Loop ---" << std::endl;
    std::cout << "Publishing sine wave commands at " << control_frequency << "Hz..." << std::endl;
    std::cout << "Data will be printed every 10 messages" << std::endl;
    std::cout << "Press Ctrl+C to stop" << std::endl;

    const double control_period = 1.0 / control_frequency;
    const int total_iterations = static_cast<int>(duration * control_frequency);

    auto& robot = GlobalRobot::getInstance();
    auto start_time = std::chrono::steady_clock::now();

    for (int iteration = 0; iteration < total_iterations; ++iteration) {
        // 计算当前时间
        auto current_time = std::chrono::steady_clock::now();
        double elapsed_seconds = std::chrono::duration<double>(current_time - start_time).count();

        // 生成正弦波控制指令
        RobotCmd cmd = generateSineWaveCommand(elapsed_seconds);

        // 发布控制指令
        bool success = robot.publishRobotCmd(cmd);
        if (!success && iteration % 100 == 0) {
            std::cout << "⚠ Warning: Failed to publish command at iteration " << iteration << std::endl;
        }

        // 每100次迭代打印一次状态
        if (iteration % 100 == 0) {
            std::cout << "Status: Iteration " << iteration << "/" << total_iterations
                     << ", Time: " << std::fixed << std::setprecision(1) << elapsed_seconds << "s"
                     << ", IMU msgs: " << imu_count_.load()
                     << ", Joint msgs: " << joint_state_count_.load() << std::endl;

            // 打印发送的完整控制指令消息
            std::cout << "Sent command:\n" << cmd << std::endl;
        }

        // 按照控制频率休眠
        std::this_thread::sleep_for(std::chrono::duration<double>(control_period));
    }

    std::cout << "✓ Control loop completed" << std::endl;
}


int main() {
    std::cout << "=== Leju SDK - Kuavo Robot Example (Class-based) ===" << std::endl;
    std::cout << "Features: IMU & Joint State Subscription (print every 10 msgs), Sine Wave Control" << std::endl;

    try {
        // 创建Kuavo机器人控制器实例
        KuavoRobotController controller(RobotVersions::KUAVO5_BASE);

        // 初始化机器人
        if (!controller.initialize()) {
            std::cerr << "❌ Failed to initialize Kuavo robot controller" << std::endl;
            return 1;
        }

        // 运行控制循环
        controller.runControlLoop(30.0, 50.0);  // 30秒，50Hz

        std::cout << "\n✅ Kuavo robot example completed successfully!" << std::endl;
        std::cout << "\nClass-based design benefits:" << std::endl;
        std::cout << "✓ Encapsulated functionality with clear separation of concerns" << std::endl;
        std::cout << "✓ Private member variables with controlled access" << std::endl;
        std::cout << "✓ Modular methods for better maintainability" << std::endl;
        std::cout << "✓ Easy to extend and modify for different robot types" << std::endl;
        std::cout << "✓ Clean resource management and error handling" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "❌ Kuavo example failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "❌ Kuavo example failed with unknown exception" << std::endl;
        return 1;
    }

    return 0;
}