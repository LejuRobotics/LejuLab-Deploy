#include <gtest/gtest.h>

#include "leju-rl-controller/runtime/input/external_interface.h"
#include "lejusdk-utils/time_utils.hpp"

namespace leju {
namespace runtime {

class ExternalInterfaceTestPeer {
 public:
  static void setVelocityLimits(ExternalInterface& interface,
                                double linear_x,
                                double linear_y,
                                double angular_z) {
    interface.velocity_limits_.max_linear_x = linear_x;
    interface.velocity_limits_.max_linear_y = linear_y;
    interface.velocity_limits_.max_angular_z = angular_z;
  }

  static void receiveVelocity(ExternalInterface& interface,
                              double linear_x,
                              double linear_y,
                              double angular_z) {
    vr::VelocityCmd cmd;
    cmd.linear_x = linear_x;
    cmd.linear_y = linear_y;
    cmd.angular_z = angular_z;
    interface.onVelocityCmd(cmd);
  }

  static void ageVelocity(ExternalInterface& interface, double age_sec) {
    const double now_sec = common::GetSteadyTimestampNs() * 1e-9;
    interface.velocity_last_rx_time_sec_.store(now_sec - age_sec);
  }
};

TEST(ExternalInterfaceVelocity, ScalesNormalizedInputToConfiguredLimits) {
  ExternalInterface interface;
  ExternalInterfaceTestPeer::setVelocityLimits(interface, 0.5, 0.3, 0.25);

  ExternalInterfaceTestPeer::receiveVelocity(interface, 0.2, -0.1, 0.15);

  const auto snapshot = interface.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_x, 0.1);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_y, -0.03);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.angular_z, 0.0375);
}

TEST(ExternalInterfaceVelocity, ClampsNormalizedInputBeforeScaling) {
  ExternalInterface interface;
  ExternalInterfaceTestPeer::setVelocityLimits(interface, 0.5, 0.3, 0.25);

  ExternalInterfaceTestPeer::receiveVelocity(interface, 2.0, -2.0, 0.4);

  const auto snapshot = interface.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_y, -0.3);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.angular_z, 0.1);
}

TEST(ExternalInterfaceVelocity, InvalidatesVelocityAfterReceiveTimeout) {
  ExternalInterface interface;
  ExternalInterfaceTestPeer::setVelocityLimits(interface, 0.5, 0.3, 0.25);
  ExternalInterfaceTestPeer::receiveVelocity(interface, 0.2, 0.0, 0.0);

  ExternalInterfaceTestPeer::ageVelocity(interface, 0.49);
  EXPECT_TRUE(interface.getSnapshot().cmd_vel.valid);

  ExternalInterfaceTestPeer::ageVelocity(interface, 0.51);
  const auto stale_snapshot = interface.getSnapshot();
  EXPECT_FALSE(stale_snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(stale_snapshot.cmd_vel.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(stale_snapshot.cmd_vel.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(stale_snapshot.cmd_vel.angular_z, 0.0);
}

TEST(ExternalInterfaceVelocity, NewCommandReactivatesAfterTimeout) {
  ExternalInterface interface;
  ExternalInterfaceTestPeer::setVelocityLimits(interface, 0.5, 0.3, 0.25);
  ExternalInterfaceTestPeer::receiveVelocity(interface, 0.2, 0.0, 0.0);
  ExternalInterfaceTestPeer::ageVelocity(interface, 0.51);
  ASSERT_FALSE(interface.getSnapshot().cmd_vel.valid);

  ExternalInterfaceTestPeer::receiveVelocity(interface, -0.1, 0.1, -0.2);
  const auto snapshot = interface.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_x, -0.05);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_y, 0.03);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.angular_z, -0.05);
}

TEST(ExternalInterfaceVelocity, ZeroCommandStopsAndRefreshesTimeout) {
  ExternalInterface interface;
  ExternalInterfaceTestPeer::setVelocityLimits(interface, 0.5, 0.3, 0.25);
  ExternalInterfaceTestPeer::receiveVelocity(interface, 0.2, 0.0, 0.0);

  ExternalInterfaceTestPeer::receiveVelocity(interface, 0.0, 0.0, 0.0);
  const auto snapshot = interface.getSnapshot();
  ASSERT_TRUE(snapshot.cmd_vel.valid);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_x, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.linear_y, 0.0);
  EXPECT_DOUBLE_EQ(snapshot.cmd_vel.angular_z, 0.0);
}

}  // namespace runtime
}  // namespace leju
