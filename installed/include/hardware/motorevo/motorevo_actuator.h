#ifndef _MOTOREVO_ACTUATOR_H_
#define _MOTOREVO_ACTUATOR_H_

#include <string>
#include <vector>
#include <map>
#include <set>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <pthread.h>
#include <cstring>
#include <iostream>

#include "canbus_sdk/canbus_sdk.h"
#include "canbus_sdk/config_parser.h"
#include "motorevo/motor_def.h"
#include "motorevo/motor_ctrl.h"
#include "ruiwo_actuator_base.h"

namespace motorevo {

class MotorevoActuator : public RuiwoActuatorBase {
public:
    /**
     * @brief 构造函数（按 CAN 总线粒度选择零点校准范围）
     * @param config_file 配置文件路径
     * @param cali_buses  要执行零点校准的 CAN 总线名称集合。
     *                    - 空集 = 不执行任何零点校准
     *                    - 典型用法:
     *                        {"bcan2","bcan3"}                         只校上肢（手臂+头）
     *                        {"bcan0","bcan1"}                         只校下肢（腿+腰）
     *                        {"bcan0","bcan1","bcan2","bcan3"}         全身
     *                    - 未在集合内的总线上的电机按非校准模式初始化（不会清零、不会重写零点文件）
     * @param control_frequency 控制频率(Hz)，默认250
     * @param single_motor_mode 单电机标定专用: 启动阶段跳过整体使能/moveToZero,
     *                          所有电机保持失能静止; calibrateSingleMotor() 只对
     *                          操作者选中的那一个电机单独做 清多圈->使能->标零->失能
     */
    MotorevoActuator(const std::string& config_file,
                     std::set<std::string> cali_buses,
                     int control_frequency = 250,
                     bool skip_head_runtime = false,
                     bool single_motor_mode = false);

    /**
     * @brief 析构函数
     */
    ~MotorevoActuator();

    /******************************************************************************/
    /*                          基类接口兼容 (MotorActuatorBase)                  */
    /******************************************************************************/
    virtual int initialize() override;
    virtual int enable() override;
    virtual int disable() override;
    virtual bool disableMotor(int motorIndex) override;
    virtual void close() override;
    // 把当前位置记作零点
    virtual void saveAsZeroPosition() override;
    // 把当前位置保存到零点文件
    virtual void saveZeroPosition() override;
    virtual void set_teach_pendant_mode(int mode) override;
    virtual void changeEncoderZeroRound(int index, double direction) override;
    virtual void adjustZeroPosition(int index, double offset) override;
    virtual std::vector<double> getMotorZeroPoints() override;
    // 单独标定一个电机: 当前位置设为零点并立即持久化, 不影响其它电机
    virtual bool calibrateSingleMotor(int index) override;

    /** @brief 对所有标定总线执行完整标定并保存到文件（用户按 'c' 确认后调用）
     *  顺序: 各总线 calibrateAllMotorsAndHold() → saveZeroToFile()
     *  @return 标定并保存是否成功
     */
    bool calibrateAllAndSave();

    virtual MotorStateDataVec get_motor_state() override;
    virtual void get_all_status(std::vector<uint8_t>& status_codes) override;
    virtual bool check_lowerlimb_fault(std::string& fault_info, bool ready_ok) override;
    virtual void set_positions(const std::vector<uint8_t> &index,
                             const std::vector<double> &positions,
                             const std::vector<double> &torque,
                             const std::vector<double> &velocity,
                             const std::vector<double> &kp_pos = {},
                             const std::vector<double> &kd_pos = {}) override;
    virtual void set_torque(const std::vector<uint8_t> &index,
                           const std::vector<double> &torque) override;
    virtual void set_velocity(const std::vector<uint8_t> &index,
                             const std::vector<double> &velocity) override;
    virtual std::vector<double> get_positions() override;
    virtual std::vector<double> get_torque() override;
    virtual std::vector<double> get_velocity() override;
    virtual void get_all_data(std::vector<double>& pos,
                              std::vector<double>& vel,
                              std::vector<double>& torque) override;
    virtual std::vector<std::string> getMotorRefNames() const override;
    
    /**
     * @brief 设置指定关节的kp_pos和kd_pos参数
     *
     * @param joint_indices 关节索引列表 (0-based)
     * @param kp_pos kp_pos值列表，如果为空则不修改
     * @param kd_pos kd_pos值列表，如果为空则不修改
     */
    virtual void set_joint_gains(const std::vector<int> &joint_indices,
        const std::vector<double> &kp_pos,
        const std::vector<double> &kd_pos) override;

    /**
    * @brief 获取指定关节的kp_pos和kd_pos参数
    *
    * @param joint_indices 关节索引列表，如果为空则返回所有关节
    * @return std::vector<std::vector<double>> 第一个vector是kp_pos，第二个是kd_pos
    */
    virtual std::vector<std::vector<double>> get_joint_gains(const std::vector<int> &joint_indices = {}) override;

    /**
     * @brief 激活 runtime_skip: 头部等 skip_head_runtime_comm 电机停止发运动帧
     *
     * 初始化阶段(使能后 hold、moveToZero)头部正常回零; 上层控制开始下发命令后
     * 由 HardwareNode::handleJoyData (Start 键按下) 调用此接口，恢复"仅使能不控制"行为。
     */
    virtual void setRuntimeSkipActive(bool active) override;
    /******************************************************************************/
    /*                          现有接口 (MotorevoActuator特有)                    */
    /******************************************************************************/
    
    
    /**
     * @brief 获取默认零点文件路径
     * @return 默认零点文件路径 $HOME/.config/lejuconfig/arms_zero.yaml
     */
    static std::string getDefaultZeroFilePath();

    /**
     * @brief 初始化执行器
     * @return int 返回0表示成功，否则返回错误码
     *  0: 成功
     *  1: 配置错误，如配置文件不存在、解析错误等
     *  2: CAN总线错误，如CAN总线初始化失败等
     *  3: 电机控制器错误，如电机控制器初始化失败等
     */
    int init();

    /**
     * @brief 重置执行器，停止线程并回收资源
     */
    void reset();

    /**
     * @brief 获取所有电机位置
     * @return 电机位置数据列表
     */
    std::vector<double> getPositions();

    /**
     * @brief 获取所有电机速度
     * @return 电机速度数据列表
     */
    std::vector<double> getVelocities();

    /**
     * @brief 获取所有电机扭矩
     * @return 电机扭矩数据列表
     */
    std::vector<double> getTorques();

    /**
     * @brief 设置目标位置、速度、扭矩和控制参数
     * @param targets 电机控制命令列表（按索引对应电机ID）
     */
    void setTargets(const std::vector<RevoMotorCmd_t>& targets);

    /**
     * @brief 设置控制线程CPU亲和性
     * @param cpu_core CPU核心编号
     * @return 设置是否成功
     */
    bool setControlThreadAffinity(int cpu_core);

    /**
     * @brief 使能所有电机
     * @return 使能是否成功
     */
    bool enableAll();

    /**
     * @brief 失能所有电机
     * @return 失能是否成功
     */
    bool disableAll();

private:
    struct MotorControlRef;
    /**
     * @brief 保存电机零点位置到文件
     */
    void saveZeroToFile();

    /**
     * @brief 构建索引与电机ID的映射关系
     */
    void buildIndexMappings(const std::vector<MotorControlRef>& motor_refs);

    // 控制线程管理
    bool startControlThread();
    // CAN FD 广播控制线程 (500Hz): bcan0/bcan1 腿部
    void canfdControlThreadFunc();
    // CAN 单帧控制线程 (250Hz): bcan2/bcan3 手臂
    void canControlThreadFunc();

    int getControlFrequency() const { return control_frequency_.load(std::memory_order_relaxed); }
    
private:
    // 控制线程相关
    std::thread canfd_control_thread_;        // CAN FD 广播控制线程 (500Hz)
    std::thread can_control_thread_;          // CAN 单帧控制线程 (250Hz)
    std::atomic<bool> running_;               // 线程运行标志
    std::atomic<int> control_frequency_;      // 控制频率(Hz)

    std::string config_file_;                 // 配置文件路径
    std::set<std::string> cali_buses_;        // 需执行零点校准的 CAN 总线名称集合(empty = 不校准)
    bool skip_head_runtime_ = false;          // 头部仅使能/去使能, 运行期不发运动帧 (来自 kuavo.json)
    bool single_motor_mode_ = false;          // 单电机标定专用: 跳过启动阶段整体使能/moveToZero

    // 多CAN总线管理
    struct CanBusGroup {
        std::string name;
        std::shared_ptr<motorevo::RevoMotorControl> motor_control;
    };

    // 电机控制引用（O(1)访问优化）
    struct MotorControlRef {
        size_t bus_group_index;      // CAN总线组索引
        motorevo::MotorId id;        // 电机ID
        std::string name;            // 电机名称
        bool ignore;                 // 被标记为忽略
        motorevo::MotorParam_t rt_params; // 运行时控制参数
    };

    // 索引与电机ID映射（按 [总线组][电机ID] 二维索引，解决多总线ID冲突）
    constexpr static size_t MAX_MOTOR_ID = 0x1F;  // 最大电机ID (1-31, 0x00为非法)
    constexpr static size_t MAX_BUS_GROUPS = 8;   // 最大CAN总线组数
    size_t id2indexs_[MAX_BUS_GROUPS][MAX_MOTOR_ID + 1]; // [bus_group][motor_id] → index
    std::vector<MotorControlRef> motor_refs_;   // 电机控制引用缓存（O(1)访问）=> index2ids
    std::vector<CanBusGroup> can_bus_groups_;  // CAN总线组列表

    struct HealthLogState {
        MotorHealthReason reason{MotorHealthReason::NONE};
        uint64_t last_log_ms{0};
    };
    std::vector<HealthLogState> health_log_states_;
    std::vector<uint8_t> health_issue_seen_;
    std::vector<MotorHealthIssue> health_issues_scratch_;
    uint64_t last_check_now_ms_{0};
    uint64_t last_suppress_log_ms_{0};
};

} // namespace motorevo

#endif
