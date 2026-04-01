#pragma once
#include <map>

#include <iostream>
#include "Eigen/Core"
#include <vector>

enum EndEffectorType
{
  none,
  jodell,
  qiangnao,
  lejuclaw,
  qiangnao_touch,
  revo2
};
enum MotorDriveType
{
  EC_MASTER,
  DYNAMIXEL,
  REALMAN,
  RUIWO,
  UNKNOWN
};

typedef struct
{
  EndEffectorType type;
  Eigen::VectorXd position;
  Eigen::VectorXd velocity;
  Eigen::VectorXd torque;
  void resize(uint8_t num_joints)
  {
    position.resize(num_joints);
    velocity.resize(num_joints);
    torque.resize(num_joints);
  }
} EndEffectorData;