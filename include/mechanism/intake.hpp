#pragma once
#include <memory>
#include "main.h"
#include "../utilities/change_detector.hpp"
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
    enum class RingColors
    {
        NONE,
        RED,
        BLUE,
    };

    class Intake
    {
    private:
        std::shared_ptr<pros::MotorGroup> motors;
        std::shared_ptr<pros::Optical> m_optical_sensor;
        std::shared_ptr<pros::Distance> m_distance_sensor;

        std::int32_t m_sort_distance;
        double m_red_bound;
        double m_blue_bound;

        bool m_sort_enabled;
        RingColors m_sort_colour;
        std::vector<RingColors> m_possession = {};

        ChangeDetector<RingColors> m_colour_state_detector;
        ChangeDetector<bool> m_ring_state_detector;

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

        void enable_sort(RingColors sortColor);
        void disable_sort();

        RingColors get_sort_colour();
        RingColors get_current_ring_colour();
        double get_possession_count();
        std::vector<RingColors> get_possession();

        void start_task();
        void stop_task();
        void set_state(IntakeState state);
        void unload();
        IntakeState get_state();
    };
} // namespace mechanism
