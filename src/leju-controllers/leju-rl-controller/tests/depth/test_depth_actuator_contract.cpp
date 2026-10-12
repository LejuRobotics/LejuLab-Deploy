#include <gtest/gtest.h>

#include "leju-rl-controller/controllers/depth_actuator_contract.h"

TEST(DepthActuatorContract, SourceMujocoAddsZeroPositionPdToActuation) {
  const auto result = leju::depth::sourceMujocoControl(
      /*actuation=*/3.0, /*q=*/0.2, /*v=*/-0.4,
      /*kp=*/10.0, /*kd=*/2.0);
  // ROS MuJoCo: ff_tau + kp*(0-q) + kd*(0-v).
  EXPECT_DOUBLE_EQ(result, 3.0 - 10.0 * 0.2 + 2.0 * 0.4);
}

TEST(DepthActuatorContract, TargetProfileKeepsActuationUnmodified) {
  const auto result = leju::depth::targetTorqueControl(3.0);
  EXPECT_DOUBLE_EQ(result, 3.0);
}

TEST(DepthActuatorContract, ExecutorRequiresReadyAction) {
  EXPECT_FALSE(leju::depth::sourceMujocoExecutorEnabled(true, false));
  EXPECT_TRUE(leju::depth::sourceMujocoExecutorEnabled(true, true));
  EXPECT_FALSE(leju::depth::sourceMujocoExecutorEnabled(false, true));
}
