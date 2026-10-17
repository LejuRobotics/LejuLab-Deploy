#pragma once

#include <array>
#include <cstddef>

namespace leju::runtime {

inline constexpr std::size_t kDexterousHandDof = 6;

// Revo2 默认张开姿态：拇指外展，其余通道张开。
inline constexpr std::array<double, kDexterousHandDof> kDefaultOpenHandPose{
    60.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// LB+X 全开 toggle 姿态（分段握拳第 0 段与张开终点）。
inline constexpr std::array<double, kDexterousHandDof> kFullyOpenHandPose{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// 注：LB+X 握拳终点不再单独定义，与倒地起身/搬运共用
// kTransportClosedHandPose（{75,75,100,100,100,100}，真机验证安全值）。

// 搬运模式分阶段握拳和退出释放姿态。
inline constexpr std::array<double, kDexterousHandDof> kTransportFourFingerGripPose{
    0.0, 0.0, 100.0, 100.0, 100.0, 100.0};
inline constexpr std::array<double, kDexterousHandDof> kTransportClosedHandPose{
    75.0, 75.0, 100.0, 100.0, 100.0, 100.0};
inline constexpr std::array<double, kDexterousHandDof> kTransportReleaseHandPose{
    30.0, 0.0, 0.0, 0.0, 0.0, 0.0};

}  // namespace leju::runtime
