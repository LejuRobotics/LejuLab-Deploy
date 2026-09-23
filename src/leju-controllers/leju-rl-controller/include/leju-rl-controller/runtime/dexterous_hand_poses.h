#pragma once

#include <array>
#include <cstddef>

namespace leju::runtime {

inline constexpr std::size_t kDexterousHandDof = 6;

// Revo2 默认张开姿态：拇指外展，其余通道张开。
inline constexpr std::array<double, kDexterousHandDof> kDefaultOpenHandPose{
    60.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// LB+X 的全开/全闭 toggle 姿态。
inline constexpr std::array<double, kDexterousHandDof> kFullyOpenHandPose{
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
inline constexpr std::array<double, kDexterousHandDof> kFullyClosedHandPose{
    100.0, 100.0, 100.0, 100.0, 100.0, 100.0};

// 搬运模式分阶段握拳和退出释放姿态。
inline constexpr std::array<double, kDexterousHandDof> kTransportFourFingerGripPose{
    0.0, 0.0, 100.0, 100.0, 100.0, 100.0};
inline constexpr std::array<double, kDexterousHandDof> kTransportClosedHandPose{
    75.0, 75.0, 100.0, 100.0, 100.0, 100.0};
inline constexpr std::array<double, kDexterousHandDof> kTransportReleaseHandPose{
    30.0, 0.0, 0.0, 0.0, 0.0, 0.0};

}  // namespace leju::runtime
