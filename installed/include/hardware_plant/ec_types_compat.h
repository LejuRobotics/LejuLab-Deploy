#ifndef _EC_TYPES_COMPAT_H_
#define _EC_TYPES_COMPAT_H_

// Standalone type definitions for EC-Master types.
// Used when EC-Master is not available (e.g., ARM aarch64 builds).
// These must stay in sync with the definitions in EcDemoApp.h.

#include <cstdint>
#include <vector>
#include <string>
#include <functional>
#include <unistd.h>

enum EcMasterType
{
  ELMO = 0,
  YD = 1,
  LEJU = 2
};

enum RobotModel
{
  KUAVO = 0,
  ROBAN2 = 1,
  KUAVO5 = 2,
  LUNBI = 3,
  ROBOT_MODEL_NUM
};

typedef struct
{
  uint8_t slave_id;
  uint8_t logical_id;
  uint8_t pdo_id;
  uint8_t physical_id;
  EcMasterType driver_type;
} MotorId_t;

typedef struct
{
  double position = 0.0;
  double velocity = 0.0;
  double torque = 0.0;
  double maxTorque = 0.0;
  double positionOffset = 0.0;
  double velocityOffset = 0.0;
  double torqueOffset = 0.0;
  double acceleration = 0.0;
  double kp = 0.0;
  double kd = 0.0;
  uint8_t status = 0;
  uint16_t status_word = 0;
  uint16_t error_code = 0;
  double torque_demand_trans = 0.0;
  double velocity_demand_raw = 0.0;
  double igbt_temperature = 0.0;
} MotorParam_t;

// Global driver_type array stub
inline EcMasterType driver_type[30] = {};

// Stub functions
inline void OsSleep(unsigned int dwMsec) { usleep(dwMsec * 1000); }
inline bool isMotorEnable() { return false; }
inline bool motorIsEnable(const uint16_t) { return false; }
inline uint8_t motorStatus(const uint16_t) { return 0; }

inline void motorGetData(const uint16_t *, const EcMasterType *, uint32_t, MotorParam_t *) {}
inline void setRobotMoudle(const int) {}
inline void motorSetPosition(const uint16_t *, const EcMasterType *, uint32_t, MotorParam_t *) {}
inline void motorSetVelocity(const uint16_t *, const EcMasterType *, uint32_t, MotorParam_t *) {}
inline void motorSetTorque(const uint16_t *, const EcMasterType *, uint32_t, MotorParam_t *) {}
inline void motorSetTorqueWithFeedback(const uint16_t *, const EcMasterType *, uint32_t, MotorParam_t *) {}
inline void setEcEncoderRange(uint32_t *, uint16_t) {}
inline void disableMotor(const uint16_t *, uint32_t) {}
inline void motorSetkp(const std::vector<int32_t> &) {}
inline void motorSetkd(const std::vector<int32_t> &) {}
inline int motorReadKp(const std::vector<uint16_t> &, EcMasterType, std::vector<int32_t> &) { return 1; }
inline int motorReadKd(const std::vector<uint16_t> &, EcMasterType, std::vector<int32_t> &) { return 1; }
inline int motorWriteKp(const std::vector<uint16_t> &, EcMasterType, const std::vector<int32_t> &) { return 1; }
inline int motorWriteKd(const std::vector<uint16_t> &, EcMasterType, const std::vector<int32_t> &) { return 1; }
inline bool saveOffset() { return false; }
inline void getMotorPositionOffset(std::vector<double> &) {}
inline void setMotorPositionOffset(const std::vector<double> &) {}

#endif // _EC_TYPES_COMPAT_H_
