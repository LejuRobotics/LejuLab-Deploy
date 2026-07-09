/**
 * @file lejusdk_vr.h
 * @brief Leju VR SDK 主头文件
 *
 * 包含 VR 控制所需的所有接口和数据类型。
 *
 * @example 基本用法
 * @code
 * #include <lejusdk-vr/lejusdk_vr.h>
 *
 * using namespace leju::vr;
 *
 * int main() {
 *     // 创建 VR API 实例
 *     KuavoVRAPI vr_api;
 *
 *     // 初始化
 *     if (!vr_api.initialize()) {
 *         std::cerr << "Failed to initialize VR API" << std::endl;
 *         return 1;
 *     }
 *
 *     // 设置手臂为外部控制模式
 *     vr_api.setArmMode(ControlMode::kExternal);
 *
 *     // 发布手臂关节指令
 *     JointTrajectoryPoint cmd;
 *     cmd.q = {0.0, 0.5, -0.3, 0.0, 0.0, 0.0, 0.0,  // 左臂
 *              0.0, -0.5, 0.3, 0.0, 0.0, 0.0, 0.0}; // 右臂
 *     vr_api.publishArmJointCmd(cmd);
 *
 *     // 发布 VR 速度指令
 *     VrVelocityCmd vr_vel;
 *     vr_vel.linear_x = 0.5;  // 前进 0.5 m/s
 *     vr_api.publishVrVelocityCmd(vr_vel);
 *
 *     return 0;
 * }
 * @endcode
 */

#ifndef LEJUSDK_VR_H_
#define LEJUSDK_VR_H_

#include "lejusdk-vr/data_types.h"
#include "lejusdk-vr/vr_api/vr_base.h"
#include "lejusdk-vr/vr_api/kuavo_vr_api.h"
#include "lejusdk-vr/vr_api/roban_vr_api.h"

#endif  // LEJUSDK_VR_H_
