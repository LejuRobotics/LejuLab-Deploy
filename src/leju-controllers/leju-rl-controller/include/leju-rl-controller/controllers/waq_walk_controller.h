#pragma once

#include <deque>
#include <atomic>

#include "leju-rl-controller/controllers/generic_rl_controller.h"

namespace leju {

class WaqWalkController final : public GenericRLController {
    public:
        WaqWalkController(const RobotVersion& version, const std::string& name);
        bool initialize() override;

    protected:
        bool loadConfig(const std::string& config_path) override;
        void reset() override;
        void computeObservation() override;
        void computeActions() override;
        bool inferActions(const array_t& observation, array_t& inferred_actions) override;
        int customObservationSize() const override {
            return frame_stack_count_ * single_obs_size_;
        }   

    private:
        array_t buildSingleObservation();
        void updatePhase(const array_t& velocity_cmd, double gait_frequency);

        int frame_stack_count_ = 10;
        int single_obs_size_ = 78;
        double cycle_time_ = 1.2;
        double cycle_time_short_ = 1.0;
        double switch_ratio_ = 1.5;
        double leg_bias_ = 0.5;
        double stance_ratio_ = 0.5;
        double phase_ = 0.0;
        double policy_dt_sec_ = 0.02;
        bool phase_initialized_ = false;
        std::atomic<double> gait_frequency_{1.0};
        std::atomic<double> fre_phase_{0.0};
        double clip_actions_ = 5000.0;  
        array_t velocity_limits_ = array_t::Zero(6);

        std::deque<array_t> obs_stack_;
};

void ForceLinkWaqWalkController();

}  // namespace leju
