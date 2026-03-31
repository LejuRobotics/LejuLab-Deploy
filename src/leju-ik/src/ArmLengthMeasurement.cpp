#include "leju-ik/ArmLengthMeasurement.h"

#include <numeric>

namespace leju {
namespace ik {

void ArmLengthMeasurement::updateMeasurement(double humanUpperArmLength,
                                            double humanLowerArmLength,
                                            const std::string& side) {
  if (side == "Left") {
    leftUpperArmLengths_.push_back(humanUpperArmLength);
    leftLowerArmLengths_.push_back(humanLowerArmLength);

    if (leftUpperArmLengths_.size() > static_cast<size_t>(armLengthNum_)) {
      leftUpperArmLengths_.erase(leftUpperArmLengths_.begin());
      leftLowerArmLengths_.erase(leftLowerArmLengths_.begin());
    }

    if (!leftUpperArmLengths_.empty()) {
      double sumUpper = std::accumulate(leftUpperArmLengths_.begin(), leftUpperArmLengths_.end(), 0.0);
      double sumLower = std::accumulate(leftLowerArmLengths_.begin(), leftLowerArmLengths_.end(), 0.0);
      avgLeftUpperArmLength_ = sumUpper / leftUpperArmLengths_.size();
      avgLeftLowerArmLength_ = sumLower / leftLowerArmLengths_.size();
    }
  } else if (side == "Right") {
    rightUpperArmLengths_.push_back(humanUpperArmLength);
    rightLowerArmLengths_.push_back(humanLowerArmLength);

    if (rightUpperArmLengths_.size() > static_cast<size_t>(armLengthNum_)) {
      rightUpperArmLengths_.erase(rightUpperArmLengths_.begin());
      rightLowerArmLengths_.erase(rightLowerArmLengths_.begin());
    }

    if (!rightUpperArmLengths_.empty()) {
      double sumUpper = std::accumulate(rightUpperArmLengths_.begin(), rightUpperArmLengths_.end(), 0.0);
      double sumLower = std::accumulate(rightLowerArmLengths_.begin(), rightLowerArmLengths_.end(), 0.0);
      avgRightUpperArmLength_ = sumUpper / rightUpperArmLengths_.size();
      avgRightLowerArmLength_ = sumLower / rightLowerArmLengths_.size();
    }
  }
}

void ArmLengthMeasurement::completeMeasurement() {
  measureArmLength_ = false;
}

}  // namespace ik
}  // namespace leju
