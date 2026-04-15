#pragma once

// Pinocchio forward declarations must be included first
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

#include <Eigen/Dense>
#include <string>
#include <vector>

namespace leju {

/**
 * @brief 手臂力矩控制器
 *
 * 使用 Pinocchio 动力学库计算手臂关节力矩：
 *   tau = G(q) + kp*(q_desired - q_measured) + kd*(v_desired - v_measured)
 * 输入输出均在 URDF（物理）空间。
 */
class ArmTorqueController {
public:
    ArmTorqueController() = default;

    /**
     * @brief 初始化：加载 URDF 并查找手臂关节索引
     * @param urdf_path URDF 文件路径
     * @param arm_joint_names 手臂关节名称列表（与 URDF 中的 joint name 一致）
     * @param kp 位置增益（手臂关节数维）
     * @param kd 速度增益（手臂关节数维）
     * @return 成功返回 true
     */
    bool init(const std::string& urdf_path,
              const std::vector<std::string>& arm_joint_names,
              const Eigen::VectorXd& kp,
              const Eigen::VectorXd& kd);

    /**
     * @brief 设置当前手臂测量状态（URDF 空间）
     * @param arm_q 手臂关节位置 [rad]
     * @param arm_v 手臂关节速度 [rad/s]
     */
    void setMeasuredState(const Eigen::VectorXd& arm_q,
                          const Eigen::VectorXd& arm_v);

    /**
     * @brief 计算手臂关节力矩：G + kp*(q_d - q_m) + kd*(v_d - v_m)
     * @param q_desired 期望手臂关节位置 [rad]（URDF 空间）
     * @param v_desired 期望手臂关节速度 [rad/s]（URDF 空间）
     * @return 手臂关节力矩 [Nm]（URDF 空间）
     */
    Eigen::VectorXd computeTorque(const Eigen::VectorXd& q_desired,
                                  const Eigen::VectorXd& v_desired);

    bool isInitialized() const { return initialized_; }
    int getArmJointCount() const { return n_arm_; }

private:
    pinocchio::Model model_;
    pinocchio::Data data_;

    Eigen::VectorXd q_full_;
    Eigen::VectorXd v_full_;
    Eigen::VectorXd kp_;  ///< 位置增益（全模型维度，仅手臂部分非零）
    Eigen::VectorXd kd_;  ///< 速度增益（全模型维度，仅手臂部分非零）

    struct JointIndex {
        int q_idx;
        int v_idx;
    };
    std::vector<JointIndex> arm_indices_;
    int n_arm_ = 0;
    bool initialized_ = false;
};

}  // namespace leju
