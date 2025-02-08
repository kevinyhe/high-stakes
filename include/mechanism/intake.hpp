#pragma once
#include <memory>
#include "main.h"
#include <mutex>

namespace mechanism
{
    enum class IntakeState
    {
        DISABLED,
        HOOK,
        WALL_STAKE,
        REVERSE,
        DEJAM
    };

    class Intake
    {
    private:
        std::shared_ptr<pros::MotorGroup> motors;
        IntakeState state = IntakeState::DISABLED;
        double dejam_start_time = 0;
        double zero_velocity_start_time = 0;
        double wall_stake_start_time = 0;
        IntakeState pre_dejam_state = IntakeState::DISABLED;
        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]()
                                     { return; });

        static std::unique_ptr<Intake> instance;
        static std::once_flag init_flag;

        // private constructor/destructor
        explicit Intake(std::shared_ptr<pros::MotorGroup> motors);
        Intake() = delete;

    public:
        ~Intake(); // If the singleton is implemented as a variable at global scope, it must have a public destructor
                   // Only public members are accessible at global scope

        // delete copy/move operations
        Intake(const Intake &) = delete;
        Intake(Intake &&) = delete;
        Intake &operator=(const Intake &) = delete;
        Intake &operator=(Intake &&) = delete;

        // Public accessor, once initialized, returns a reference
        static Intake &get_instance()
        {
            if (!instance)
            {
                throw std::runtime_error("Intake not initialized. Call initialize() first.");
            }
            return *instance;
        }

        // initialization function, call this once to create the instance
        static void initialize(std::shared_ptr<pros::MotorGroup> motors)
        {
            std::call_once(init_flag, [&]()
                           { instance.reset(new Intake(motors)); });
        }

        void start_task();
        void stop_task();
        void set_state(IntakeState state);
        void unload();
        IntakeState get_state();
    };
} // namespace mechanism
