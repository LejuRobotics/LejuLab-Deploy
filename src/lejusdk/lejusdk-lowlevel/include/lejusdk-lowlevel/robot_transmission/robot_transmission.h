#ifndef LEJUSDK_TRANSMISSION_ROBOT_TRANSMISSION_H_
#define LEJUSDK_TRANSMISSION_ROBOT_TRANSMISSION_H_

#include "lejusdk-lowlevel/leju_sdk.h"

namespace leju {
namespace transmission {

class RobotTransmissionImpl;

class RobotTransmission {
public:
   using vector_t = std::vector<double>;
   explicit RobotTransmission(RobotVersion robot_version);
   ~RobotTransmission() = default;

   vector_t joint_to_motor_position(const vector_t &q);
   vector_t joint_to_motor_velocity(const vector_t &q, const vector_t &p, const vector_t &v);
   vector_t joint_to_motor_torque(const vector_t &q, const vector_t &p, const vector_t &t);

   vector_t motor_to_joint_position(const vector_t &p);
   vector_t motor_to_joint_velocity(const vector_t &q, const vector_t &p, const vector_t &dp);
   vector_t motor_to_joint_torque(const vector_t &q, const vector_t &p, const vector_t &c);

private:
   RobotTransmissionImpl* pimpl_;
};

} // namespace transmission
} // namespace leju

#endif // LEJUSDK_TRANSMISSION_ROBOT_TRANSMISSION_H_