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
        SORT,
        HOOK,
        FIRST_HOOK,
        SECOND_HOOK,
        WALL_STAKE,
        REVERSE,
        DEJAM
    };

    class Intake
    {
    public:
        enum class RingColours
        {
            NONE,
            RED,
            BLUE,
        };

    private:
        std::shared_ptr<pros::Motor> m_motor;
        std::shared_ptr<pros::Optical> m_optical_sensor;
        std::shared_ptr<pros::Distance> m_distance_sensor;

        std::int32_t m_sort_distance;
        double m_red_bound;
        double m_blue_bound;

        bool m_sort_enabled;
        bool m_stop_next_ring_flag = false;
        RingColours m_sort_colour;
        std::vector<RingColours> m_possession = {};

        ChangeDetector<RingColours> m_colour_state_detector;
        ChangeDetector<bool> m_ring_state_detector;

        IntakeState state = IntakeState::DISABLED;
        double dejam_start_time = 0;
        double sort_start_time = 0;
        double zero_velocity_start_time = 0;
        double wall_stake_start_time = 0;
        IntakeState pre_dejam_state = IntakeState::DISABLED;
        IntakeState pre_sort_state = IntakeState::DISABLED;
        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]()
                                     { return; });

        static std::unique_ptr<Intake> instance;
        static std::once_flag init_flag;

        // private constructor/destructor
        explicit Intake(std::shared_ptr<pros::Motor> m_motor, std::shared_ptr<pros::Optical> optical_sensor, std::shared_ptr<pros::Distance> distance_sensor, std::int32_t sort_distance, double red_bound, double blue_bound);
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
        static void initialize(std::shared_ptr<pros::Motor> m_motor, std::shared_ptr<pros::Optical> optical_sensor, std::shared_ptr<pros::Distance> distance_sensor, std::int32_t sort_distance, double red_bound, double blue_bound)
        {
            std::call_once(init_flag, [&]()
                           { instance.reset(new Intake(m_motor, optical_sensor, distance_sensor, sort_distance, red_bound, blue_bound)); });
        }

        void enable_sort(RingColours sortColor);
        void disable_sort();

        RingColours get_sort_colour();
        RingColours get_current_ring_colour();
        double get_possession_count();
        std::vector<RingColours> get_possession();

        void enable_stop_next_ring();
        void disable_stop_next_ring();

        void start_task();
        void stop_task();
        void set_state(IntakeState state);
        void unload();
        IntakeState get_state();
    };
} // namespace mechanism
