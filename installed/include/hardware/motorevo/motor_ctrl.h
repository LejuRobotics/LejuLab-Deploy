#ifndef _REVOMOTOR_CTRL_H_
#define _REVOMOTOR_CTRL_H_
#include <atomic>
#include <string>
#include <map>
#include <vector>
#include <mutex>
#include <map>
#include "motorevo/motor_def.h"
#include "canbus_sdk/canbus_sdk.h"

namespace motorevo {

class RevoMotor {
public:
    RevoMotor(uint8_t canbus_id, MotorId id, MotorMode mode);
    RevoMotor(const RevoMotor& other);
    RevoMotor(RevoMotor&& other) noexcept;
    RevoMotor& operator=(const RevoMotor& other);
    RevoMotor& operator=(RevoMotor&& other) noexcept;
    virtual ~RevoMotor();

    ///////////////////////////////////////////////////////////////
    /*** Motor Info ***/

    /** @brief 获取电机的ID */
    uint32_t getId() const;

    /** @brief  获取电机运行状态
     *  @return 电机的运行状态， Rest State（休眠模式），Motor State（运行模式）
     */
    MotorState getState() const;

    /** @brief  获取电机控制模式
     *  @return 控制模式
     */
    MotorMode getControlMode() const;

    ///////////////////////////////////////////////////////////////
    /*** Motor Feedback ***/

    /** @brief  获取电机位置
     *  @return 位置(rad)，原始数据
     */
    float position() const;

    /** @brief  获取电机当前力矩
     *  @return 扭矩(N·m)，原始数据
     */
    float torque() const;

    /** @brief  获取电机当前速度
     *  @return 速度(rad/s)，原始数据
     */
    float velocity() const;

    /** @brief  获取电机温度
     *  @return 温度(℃)
     */
    uint8_t temperature() const;

    /** @brief  获取电机错误代码
     *  @return 错误代码
     */
    MotorErrCode getErrorCode() const;
    
    ///////////////////////////////////////////////////////////////
    /*** Motor Control ***/
    /* WARN: controlPTM, controlTorque, controlVelocity 等函数调用需要和 MotorMode 匹配，且和实际电机设置的控制模式一致
     *       !!! 否则！会出大问题 !!!
     *       !!! 否则！会出大问题 !!!
     *       !!! 否则！会出大问题 !!!
     */

    /** @brief  发送休眠模式指令
     *  @return 发送状态
     */
    bool enterRestState();

    /** @brief  发送电机运行模式指令
     *  @return 发送状态
     */
    bool enterMotorState();

    /** @brief  发送设置电机零位指令
     *  @return 发送状态
     */ 
    bool setZeroPosition();
    
    /** @brief  发送多圈编码器清零指令
     *  @return 发送状态
     */
    bool multiTurnZero();

    /** @brief 在 P-T-M 力位混合模式下运行电机
     *  @param pos θ_ref 位置
     *  @param vel V_ref 速度
     *  @param torque T_ref 力矩
     *  @param pos_kp 位置比例增益
     *  @param pos_kd 位置微分增益
     *  @return 成功/失败
     */
    bool controlPTM(float pos, float vel, float torque, float pos_kp, float pos_kd);

    /** @brief 在 Velocity 速度模式下运行电机
     *  @param vel V_ref 速度
     *  @param vel_kp 速度比例增益
     *  @param vel_ki 速度积分增益
     *  @param vel_kd 速度微分增益
     *  @return 成功/失败
     */
    bool controlVelocity(float vel, float vel_kp, float vel_ki, float vel_kd);

    /** @brief 在Torque 力矩模式下运行电机
     *  @param torque T_ref 力矩
     *  @return 成功/失败
     */
    bool controlTorque(float torque);

    ///////////////////////////////////////////////////////////////
    /*** CAN FD 广播协议编码 (不直接发送, 只编码8字节到buffer) ***/

    void encodeMitFrame(uint8_t buf[8], float pos, float vel, float torque, float kp, float kd) const;
    static void encodeControlFrame(uint8_t buf[8], uint8_t cmd);
    static void encodePosVelFrame(uint8_t buf[8], float pos, float vel);
    static void encodeVelFrame(uint8_t buf[8], float vel);
    bool receiveFeedbackFd(const FeedbackFrameFd& frame, MotorErrCode errcode = MotorErrCode::NO_FAULT);

    ///////////////////////////////////////////////////////////////
    /*** Motor Helper ***/

    bool receiveFeedback(const FeedbackFrame& frame);

    ///////////////////////////////////////////////////////////////
private:
    struct data_t;
    MotorState          state_;            // 电机运行状态
    MotorMode           ctrl_mode_;        // 电机控制模式
    data_t*             data_;             // 电机原始数据
    uint8_t             canbus_id_;        // CAN总线ID
    MotorId             id_;               // 电机ID
};


struct RevoJointSnapshot {
    MotorId id;
    float position;
    float velocity;
    float torque;
};

enum class MotorHealthReason : uint8_t {
    NONE = 0,
    REPORTED_FAULT,
    FEEDBACK_TIMEOUT,
    UNEXPECTED_DISABLED,
};

struct MotorHealthIssue {
    MotorId id{0};
    MotorHealthReason reason{MotorHealthReason::NONE};
    uint64_t feedback_age_ms{0};
    uint16_t status_word{0};
    bool status_word_valid{false};
    MotorErrCode fault_code{MotorErrCode::NO_FAULT};
};

struct MotorHealthSummary {
    size_t armed_count{0};
    size_t timed_out_armed_count{0};
};

class RevoMotorControl {
public:
    /** @brief 构造RevoMotor控制器
     *  @param canbus_name CAN总线名称
     */
    RevoMotorControl(const std::string& canbus_name, MotorProtocol protocol = MotorProtocol::CAN_SINGLE_FRAME);

    /** @brief 初始化电机控制器
     *  @param motor_configs 电机配置列表
     *  @param calibrate 是否进行零点校准
     *  @param move_to_zero 是否执行归零动作 (默认 true)
     *  @param enable_all 是否整体使能所有电机 (默认 true); 单电机标定模式下传 false,
     *                     所有电机保持失能, 由 calibrateSingleMotor() 单独处理目标电机
     *  @return 初始化是否成功
     */
    bool init(const std::vector<RevoMotorConfig_t> &motor_configs, bool calibrate, bool move_to_zero = true, bool enable_all = true);

    /** @brief 使能所有电机 */
    bool enableAll();
    
    /** @brief 失能所有电机 */
    bool disableAll();
    
    /** @brief 清零所有电机的多圈编码器 */
    void multiTurnZeroAll();

    /** @brief 使能指定电机
     *  @param id 电机ID
     *  @param timeout_ms 等待超时时间(毫秒)，默认100ms
     *  @return 返回值含义：
     *          - 0: 使能成功
     *          - 1: 操作超时
     *          - 2: 发送消息失败
     *          - 3: 电机不存在
     *          - 负数: 电机故障码的负数 => -MotorErrCode
     */
    int enableMotor(MotorId id, int timeout_ms = 500);

    /** @brief 失能指定电机
     *  @param id 电机ID
     *  @param timeout_ms 等待超时时间(毫秒)，默认100ms
     *  @return 返回值含义：
     *          - 0: 失能成功
     *          - 1: 操作超时
     *          - 2: 发送消息失败
     *          - 3: 电机不存在
     *          - 负数: 电机故障码的负数 => -MotorErrCode
     */
    int disableMotor(MotorId id, int timeout_ms = 500);

    /** @brief 设置0-torque模式
     *  @param enable 是否启用0-torque模式
     */
    void setZeroTorqueMode(bool enable);

    /** @brief 激活/关闭 runtime_skip 电机的运动帧跳过
     *  @param active true 后 config.runtime_skip 的电机不再发 PTM/MIT 运动帧
     *  @note 默认 false: 初始化阶段(使能后 hold、moveToZero)头部等 runtime_skip
     *        电机正常收帧回零; 上层控制开始下发命令后置 true, 恢复"仅使能不控制"
     */
    void setRuntimeSkipActive(bool active);

    /** @brief 多圈编码器清零
     *  @param id 电机ID
     *  @param timeout_ms 等待超时时间(毫秒)，默认100ms
     *  @return 清零是否成功
     */
    bool multiTurnZero(MotorId id, int timeout_ms = 100);

    /** @brief 设置电机配置
     *  @param id 电机ID
     *  @param config 新的电机配置
     *  @return 设置是否成功
     */
    bool setMotorConfig(MotorId id, const RevoMotorConfig_t& config);

    
    /** @brief 获取指定电机的配置
     *  @param id 电机ID
     *  @return 电机配置，如果电机不存在则返回默认配置
     */
    RevoMotorConfig_t getMotorConfig(MotorId id) const;

    /** @brief 获取所有电机的零点偏移值
     *  @return 电机ID到零点偏移值的映射表(rad)
     */
    std::map<MotorId, float> getZeroOffsets();

    /** @brief 单独标定一个电机: 将该电机当前反馈位置设为零点偏移
     *  @param id 电机ID
     *  @param timeout_ms 等待反馈帧超时时间(毫秒)
     *  @return 标定是否成功 (电机不存在/未收到反馈/偏移超出安全范围时返回 false)
     *  @note 只修改该电机自身的 zero_offset, 不触碰同总线其它电机
     */
    bool calibrateSingleMotor(MotorId id, int timeout_ms = 1000);

    /** @brief 执行完整标定流程（用户确认后调用）
     *  顺序: disableAll → multiTurnZeroAll → enableAll → waitForFeedback → calibrateMotors → hold
     *  @return 标定是否成功
     *  @note 替代 init() 中原 calibrate=true 时的 multiTurnZeroAll + calibrateMotors，
     *        将不可逆硬件操作从 init 阶段移到用户确认后执行，避免 Ctrl+C 退出时硬件已清零但零点文件未保存
     */
    bool calibrateAllMotorsAndHold();

    /** @brief 获取所有电机原始位置
     *  @return 电机ID到原始位置的映射表(未处理零点和方向)
     */
    std::map<MotorId, float> getRawPositions();

    /** @brief 获取所有电机原始速度
     *  @return 电机ID到原始速度的映射表(未处理方向)
     */
    std::map<MotorId, float> getRawVelocities();

    /** @brief 获取所有电机原始力矩
     *  @return 电机ID到原始力矩的映射表(未处理方向)
     */
    std::map<MotorId, float> getRawTorques();

    /** @brief 获取所有电机经过零点和方向计算后的位置
     *  @return 电机ID到处理后位置的映射表(已应用零点偏移和方向修正)
     */
    std::map<MotorId, float> getPositions();

    /** @brief 获取所有电机当前速度
     *  @return 电机ID到速度的映射表
     */
    std::map<MotorId, float> getVelocities();

    /** @brief 获取所有电机当前力矩
     *  @return 电机ID到力矩的映射表
     */
    std::map<MotorId, float> getTorques();

    /** @brief 一次遍历获取所有电机的位置、速度、力矩数据
     *  @param out 输出缓冲区（由调用方复用，避免堆分配）
     */
    void getAllJointData(std::vector<RevoJointSnapshot>& out) const;

    /** @brief 获取所有电机的状态信息
     *  @return 电机ID到运行状态的映射表
     */
    std::map<MotorId, MotorState> getMotorStates();

    /** @brief 获取所有电机的故障码
     *  @return 电机ID到故障码的映射表
     */
    std::map<MotorId, MotorErrCode> getFaultCodes();

    /** @brief 快速检查是否有任一电机存在故障码 (无堆分配, 可在实时循环中调用) */
    bool hasAnyFault() const;

    /** @brief 获取最后一次收到CAN反馈帧的时间戳 (ms) */
    uint64_t getLastFeedbackTimeMs() const { return last_feedback_time_ms_.load(); }

    /**
     * @brief 收集当前电机健康问题
     * @return 当前 armed 数量及其中反馈超时数量
     */
    MotorHealthSummary getMotorHealthIssues(uint64_t now_ms,
                                            bool watchdog_allowed,
                                            uint64_t feedback_timeout_ms,
                                            uint8_t disabled_confirm_frames,
                                            uint8_t feedback_confirm_frames,
                                            bool suppress_feedback_timeout,
                                            std::vector<MotorHealthIssue>& out) const;

    /** @brief 发送disable命令后跳过反馈确认，避免失联总线逐个等待超时 */
    void setSkipDisableWait(bool skip) { skip_disable_wait_.store(skip); }
    bool skipDisableWait() const { return skip_disable_wait_.load(); }

    /** @brief 设置所有电机的目标位置、速度和力矩
     *  @param targets 电机ID到目标控制参数的映射表
     *  @note 此函数只更新目标值，不发送到电机，需要调用write()函数发送
     */
    void setTargetPositions(const std::map<MotorId, RevoMotorCmd_t>& targets);

    /** @brief 发送电机控制指令到所有电机
     *  @note 只有当目标更新标志为true时才会发送，发送后重置标志
     */
    void write();

    MotorProtocol getProtocol() const { return protocol_; }

    /** @brief 诊断: 单帧协议 write() 统计信息 */
    struct SingleFrameWriteStats {
        uint64_t hit;            // write() 命中 target_updated_=true, 真实发送
        uint64_t miss;           // write() 未命中, 空转
        uint64_t samples;        // writeSingleFrame 总调用次数 (= hit)
        uint64_t total_us;       // writeSingleFrame 累计耗时 us
        uint64_t max_us;         // writeSingleFrame 单次最大耗时 us
    };

    /** @brief 诊断: 读取 writeSingleFrame 统计 (非重置读) */
    SingleFrameWriteStats getSingleFrameWriteStats() const;

    // 区分谁触发了 write: setTargets 路径 vs canControlThread 路径
    std::atomic<bool> write_from_setTargets_{false};
    std::atomic<uint64_t> hit_from_setTargets_{0};
    std::atomic<uint64_t> hit_from_canThread_{0};

    /** @brief 析构函数，清理资源 */
    ~RevoMotorControl();

private:
    // 指令操作
    enum class Operation {
        IDLE,           // 空闲
        ENABLE,         // 使能
        DISABLE,        // 失能
        MULTI_TURN_ZERO // 多圈清零
    };

    enum class OperationStatus {
        PENDING,        // 等待确认
        SUCCESS,        // 成功
        TIMEOUT,        // 超时
        FAILED          // 失败
    };
    // 更新电机控制命令
    void updateMotorCmd(MotorId id, double pos, double vel, double torque, double kp, double kd);

    // 从当前位置插值到零点
    void moveToZero(float zero_timeout = 1.0f);
    // 校准零点
    bool calibrateMotors();

    /** @brief CANFD_BROADCAST 协议专用: 只对目标电机的槽位下发控制命令(使能/失能/清多圈/清故障),
     *         其余电机的槽位统一填 kCmdDisable(失能), 不影响目标电机之外的整体行为语义
     *  @note 单电机标定场景下调用前提是同总线其它电机本就处于失能状态, 详见 calibrateSingleMotor()
     */
    bool sendSingleSlotControlFrame(MotorId target_id, uint8_t target_cmd);

    // CAN消息回调函数
    static void internalMessageCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);

    // TEF事件回调函数
    static void internalTefEventCallback(canbus_sdk::CanMessageFrame* frame, const canbus_sdk::CallbackContext* context);

    void writeSingleFrame();
    void writeBroadcast();

    // 等待操作状态完成
    /**
     * @brief 等待电机操作完成并检查状态
     * @param id 电机ID
     * @param expected_op 期望的操作类型
     * @param expected_status 期望的操作状态
     * @param timeout_ms 超时时间(毫秒)
     * @param operation_name 操作名称(用于日志)
     * @return 返回值含义：
     *         - 0: 操作成功完成
     *         - 1: 操作超时
     *         - 负数: 电机故障码的负数
     */
    int waitForOperationStatus(MotorId id, Operation expected_op, OperationStatus expected_status, int timeout_ms, const char* operation_name);

    // 等待所有电机第一个反馈到达
    bool waitForAllMotorsFeedback(int timeout_ms);

    // 初始化CAN总线并注册电机设备
    bool initializeCanBusAndDevices(const std::vector<RevoMotorConfig_t>& motor_configs);
    
private:
    // 回调上下文
    canbus_sdk::CallbackContext msg_callback_context_;
    canbus_sdk::CallbackContext tef_callback_context_;

    struct MotorCtrlData {
        std::atomic<Operation> operation{Operation::IDLE};              // 当前操作
        std::atomic<OperationStatus> operation_status{OperationStatus::SUCCESS}; // 操作状态
        std::atomic<bool> feedback_received{false};          // 是否收到反馈
        std::atomic<uint64_t> last_feedback_time_ms{0};      // 最近一次有效反馈时间
        std::atomic<bool> ever_received_feedback{false};     // 不受 startOperation() 清零影响
        std::atomic<uint16_t> last_status_word{0};           // 仅 CAN FD 有效
        std::atomic<bool> status_word_valid{false};          // 经典 CAN 为 false
        std::atomic<uint8_t> disabled_feedback_count{0};     // 连续 enabled=0 反馈数
        mutable std::atomic<uint8_t> feedback_timeout_streak{0}; // 连续反馈超时采样数
        mutable std::atomic<uint8_t> feedback_suppress_run{0};   // 连续迟到抑制周期数
        ////////////////////////////////////////////////
        RevoMotor motor;                  // 电机实例
        RevoMotorConfig_t config;         // 电机配置
        RevoMotorCmd_t cmd;               // 电机控制命令
        std::atomic<MotorErrCode> fault_code;  // 故障代码
        mutable std::mutex cmd_mutex;     // 保护cmd字段的互斥锁

        MotorCtrlData(bool fr, RevoMotor&& m, const RevoMotorConfig_t& c)
            : operation(Operation::IDLE), operation_status(OperationStatus::SUCCESS),
              feedback_received(fr), motor(std::move(m)), config(c),
              cmd{0.0, 0.0, 0.0, 0.0, 0.0}, fault_code{MotorErrCode::NO_FAULT}, cmd_mutex() {}

        bool isEnabled() const {
            if(fault_code != MotorErrCode::NO_FAULT) return false;  // 有故障码
            return operation.load() == Operation::ENABLE
                && feedback_received.load()
                && (operation_status.load() == OperationStatus::SUCCESS || operation_status.load() == OperationStatus::PENDING) ;
        }

        // 线程安全地获取电机命令
        RevoMotorCmd_t getCmd() const {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            return cmd;
        }

        // 线程安全地设置电机命令
        void setCmd(const RevoMotorCmd_t& new_cmd) {
            std::lock_guard<std::mutex> lock(cmd_mutex);
            cmd = new_cmd;
        }

        // 开始新的操作 (线程安全)
        void startOperation(Operation op) {
            operation.store(op);
            operation_status.store(OperationStatus::PENDING);
            feedback_received.store(false);
            if (op == Operation::ENABLE) {
                disabled_feedback_count.store(0, std::memory_order_release);
                feedback_timeout_streak.store(0, std::memory_order_release);
                feedback_suppress_run.store(0, std::memory_order_release);
            }
        }

        // 更新操作状态 (线程安全)
        void updateOperationStatus(OperationStatus status) {
            operation_status.store(status);
        }

        MotorErrCode getFaultCode() const {
            return fault_code.load();
        }

        bool expectsEnabled() const {
            return operation.load() == Operation::ENABLE
                && operation_status.load() == OperationStatus::SUCCESS;
        }
    };
    std::atomic<bool> target_updated_{false};
    std::atomic<bool> broadcast_ready_{false};   // 首次 setTargets 后置 true，CANFD 才开始发 MIT 帧
    std::atomic<bool> zero_torque_mode_{false};  // 0-torque模式标志
    std::atomic<bool> runtime_skip_active_{false};  // runtime_skip 生效标志: 初始化回零阶段为 false, 上层控制开始后置 true

    // ===== 诊断: 单帧协议 write() 统计 (relaxed atomic, 每次几 ns) =====
    std::atomic<uint64_t> single_write_hit_{0};
    std::atomic<uint64_t> single_write_miss_{0};
    std::atomic<uint64_t> single_write_total_us_{0};
    std::atomic<uint64_t> single_write_max_us_{0};
    std::string canbus_name_;  // CAN总线名称
    std::atomic<uint64_t> last_feedback_time_ms_{0};  // 最后收到CAN反馈帧的时间戳(ms)
    std::atomic<bool> skip_disable_wait_{false};       // 失联时仍发disable，仅跳过反馈确认等待
    MotorProtocol protocol_;
    uint8_t canfd_bus_id_{0};
    std::map<MotorId, MotorCtrlData> motor_ctrl_datas_;

};

} // namespace motorevo
#endif
