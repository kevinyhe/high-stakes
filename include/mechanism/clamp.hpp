#pragma once
#include "main.h"
#include "config.hpp"
#include "device/pneumatic.hpp"

namespace mechanism
{
    class Clamp
    {
    private:
        std::shared_ptr<Pneumatic> m_pneumatic;
        std::shared_ptr<pros::Distance> m_distance;

        bool m_autoclamp = false;
        double m_autoclamp_threshold;

        bool m_is_clamped = false;

        pros::Mutex mutex;
        bool task_on_flag = false;
        pros::Task task = pros::Task([]()
                                     { return; });

        static std::unique_ptr<Clamp> instance;
        static std::once_flag init_flag;
        // Private constructor/destructor
        explicit Clamp(std::shared_ptr<Pneumatic> pneumatic, std::shared_ptr<pros::Distance> distance, double autoclamp_threshold);
        Clamp() = delete;

    public:
        ~Clamp();
        // disable copy/move -- this is a Singleton
        Clamp(const Clamp &) = delete;
        Clamp(Clamp &&) = delete;
        Clamp &operator=(const Clamp &) = delete;
        Clamp &operator=(Clamp &&) = delete;

        // Singleton accessor
        static void initialize(std::shared_ptr<Pneumatic> pneumatic, std::shared_ptr<pros::Distance> distance, double autoclamp_threshold)
        {
                std::call_once(init_flag, [&]()
                               { instance.reset(new Clamp(pneumatic, distance, autoclamp_threshold)); });
        }

        // Access method
        static Clamp &get_instance()
        {
            if (!instance)
            {
                throw std::runtime_error("Clamp not initialized. Call initialize() first.");
            }
            return *instance;
        }

        // Core functionality
        void start_task();
        void stop_task();
        void extend();
        void retract();
        void toggle();

        void set_autoclamp(bool autoclamp);
        bool get_value();
        bool get_autoclamp();
        bool is_clamped();
        };
} // namespace mechanism