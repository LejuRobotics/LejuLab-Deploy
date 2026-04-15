#include <pinocchio/fwd.hpp>

#include "leju-rl-controller/rl/arm_torque_controller.h"
#include "leju-rl-controller/rl_log.h"

#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace leju {

bool ArmTorqueController::init(const std::string& urdf_path,
                                 const std::vector<std::string>& arm_joint_names,
                                 const Eigen::VectorXd& kp,
                                 const Eigen::VectorXd& kd) {
    try {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
    } catch (const std::exception& e) {
        RL_LOG_FAILURE("ArmTorqueController: failed to load URDF: %s\n  %s",
                       urdf_path.c_str(), e.what());
        return false;
    }

    q_full_ = Eigen::VectorXd::Zero(model_.nq);
    v_full_ = Eigen::VectorXd::Zero(model_.nv);

    arm_indices_.clear();
    for (const auto& name : arm_joint_names) {
        bool found = false;
        int q_idx = 0, v_idx = 0;
        for (size_t i = 1; i < model_.names.size(); ++i) {
            if (model_.names[i] == name) {
                arm_indices_.push_back({q_idx, v_idx});
                found = true;
                break;
            }
            q_idx += model_.joints[i].nq();
            v_idx += model_.joints[i].nv();
        }
        if (!found) {
            RL_LOG_FAILURE("ArmTorqueController: joint '%s' not found in URDF", name.c_str());
            return false;
        }
    }

    n_arm_ = static_cast<int>(arm_indices_.size());

    // 校验 kp/kd 维度
    if (kp.size() != n_arm_ || kd.size() != n_arm_) {
        RL_LOG_FAILURE("ArmTorqueController: kp/kd size (%d/%d) != arm joints (%d)",
                       static_cast<int>(kp.size()), static_cast<int>(kd.size()), n_arm_);
        return false;
    }

    // 构建全模型维度的 kp/kd（仅手臂部分非零）
    kp_ = Eigen::VectorXd::Zero(model_.nv);
    kd_ = Eigen::VectorXd::Zero(model_.nv);
    for (int i = 0; i < n_arm_; ++i) {
        kp_[arm_indices_[i].v_idx] = kp[i];
        kd_[arm_indices_[i].v_idx] = kd[i];
    }

    initialized_ = true;

    RL_LOG_SUCCESS("ArmTorqueController initialized: %d arm joints, model nq=%d nv=%d",
                   n_arm_, model_.nq, model_.nv);
    RL_LOG_INFO("  kp: %s", [&]() {
        std::ostringstream oss; oss << kp.transpose(); return oss.str();
    }().c_str());
    RL_LOG_INFO("  kd: %s", [&]() {
        std::ostringstream oss; oss << kd.transpose(); return oss.str();
    }().c_str());
    return true;
}

void ArmTorqueController::setMeasuredState(const Eigen::VectorXd& arm_q,
                                             const Eigen::VectorXd& arm_v) {
    if (!initialized_ || arm_q.size() != n_arm_) return;

    for (int i = 0; i < n_arm_; ++i) {
        q_full_[arm_indices_[i].q_idx] = arm_q[i];
        v_full_[arm_indices_[i].v_idx] = arm_v[i];
    }
}

Eigen::VectorXd ArmTorqueController::computeTorque(
    const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& v_desired) {
    if (!initialized_) {
        return Eigen::VectorXd::Zero(n_arm_);
    }

    // NaN 检查
    if (q_desired.hasNaN() || v_desired.hasNaN() ||
        q_full_.hasNaN() || v_full_.hasNaN()) {
        RL_LOG_WARNING("ArmTorqueController: NaN detected in input");
        return Eigen::VectorXd::Zero(n_arm_);
    }

    // 构建全模型维度的期望状态
    Eigen::VectorXd q_desired_full = Eigen::VectorXd::Zero(model_.nq);
    Eigen::VectorXd v_desired_full = Eigen::VectorXd::Zero(model_.nv);
    for (int i = 0; i < n_arm_; ++i) {
        q_desired_full[arm_indices_[i].q_idx] = q_desired[i];
        v_desired_full[arm_indices_[i].v_idx] = v_desired[i];
    }

    // 计算动力学项（基于测量状态）
    pinocchio::computeAllTerms(model_, data_, q_full_, v_full_);
    const Eigen::VectorXd& G = data_.g;

    // tau = G + kp*(q_desired - q_measured) + kd*(v_desired - v_measured)
    Eigen::VectorXd q_error = q_desired_full - q_full_;
    Eigen::VectorXd v_error = v_desired_full - v_full_;
    Eigen::VectorXd tau = G + kp_.asDiagonal() * q_error + kd_.asDiagonal() * v_error;

    // 提取手臂部分的力矩
    Eigen::VectorXd arm_tau(n_arm_);
    for (int i = 0; i < n_arm_; ++i) {
        arm_tau[i] = tau[arm_indices_[i].v_idx];
    }
    return arm_tau;
}

}  // namespace leju
