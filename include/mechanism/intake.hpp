
#pragma once

#include <memory>
#include "pros/rtos.hpp"

#include "main.h"

namespace mechanism
{
    enum class IntakeState
    {
        DISABLED,
        HOOD,
        WALL_STAKE,
        REVERSE,
        DEJAM
    };

    class Intake
    {
    private:
        std::shared_ptr<pros::MotorGroup> motors;

        IntakeState state = IntakeState::DISABLED;
        double dejam_start_time = -1;
        IntakeState pre_dejam_state = IntakeState::DISABLED;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]()
                                     { return; });

    public:
        Intake(std::shared_ptr<pros::MotorGroup> motors);

        /**
         * @brief Start the task
         */
        void start_task();
        /**
         * @brief Stop the task
         */
        void stop_task();

        /**
         * @brief Set the state of the mechanism
         *
         * @param state The new state
         */
        void set_state(IntakeState state);

        /**
         * @brief Get the state of the mechanism
         *
         * @return RingMechState The current state
         */
        IntakeState get_state();
    };
} // namespace mechanism
