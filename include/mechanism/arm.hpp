#pragma once

#include <memory>
#include "main.h"
#include "controller/pid.hpp"

namespace mechanism
{
    struct ArmTargetConfig
    {
        double load;
        double alliance_stake;
        double ladder_touch;
        double neutral_stake;
    };

    enum class ArmState
    {
        DISABLED,
        LOAD,
        ALLIANCE_STAKE,
        LADDER_TOUCH,
        NEUTRAL_STAKE
    };

    class Arm
    {
    private:
        std::shared_ptr<pros::Motor> motors;
        std::shared_ptr<pros::Rotation> arm_rotation_sensor;

        std::shared_ptr<PID> arm_pid;
        ArmTargetConfig target_config;
        double kG;

        ArmState state = ArmState::DISABLED;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]()
                                     { return; });

        static std::unique_ptr<Arm> instance;
        // Private constructor/destructor
        explicit Arm(std::shared_ptr<pros::Motor> motors,
            std::shared_ptr<pros::Rotation> arm_rotation_sensor,
            std::shared_ptr<PID> arm_pid,
            ArmTargetConfig target_config,
            double kG);
        Arm() = delete;

    public:
        ~Arm();
        // disable copy/move -- this is a Singleton
        Arm(const Arm &) = delete;
        Arm(Arm &&) = delete;
        Arm &operator=(const Arm &) = delete;
        Arm &operator=(Arm &&) = delete;

        // Singleton accessor
        static void initialize(std::shared_ptr<pros::Motor> motors,
                               std::shared_ptr<pros::Rotation> arm_rotation_sensor,
                               std::shared_ptr<PID> arm_pid,
                               ArmTargetConfig arm_target_config,
                               double kG)
        {
            if (!instance)
            {
                instance = std::unique_ptr<Arm>(new Arm(motors,
                                                        arm_rotation_sensor,
                                                        arm_pid,
                                                        arm_target_config,
                                                        kG));
            }
        }

        // Access method
        static Arm &get_instance()
        {
            if (!instance)
            {
                throw std::runtime_error("Arm not initialized. Call initialize() first.");
            }
            return *instance;
        }

        // Core functionality
        void start_task();
        void stop_task();
        void set_state(ArmState state);
        ArmState get_state();
        bool is_loading();
    };
} // namespace mechanism