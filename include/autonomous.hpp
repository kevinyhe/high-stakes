#pragma once

#include "main.h"

void red_mogo();
void blue_mogo();
void red_ring();
void blue_ring();
void red_awp();
void blue_awp();
void prog_skills();
void findTrackingCenter(int turnVoltage, uint32_t time);
class AutonSelector
{
public:
    enum class AutonRoutine
    {
        RED_MOGO,
        BLUE_MOGO,
        RED_RING,
        BLUE_RING,
        PROG_SKILLS,
        DEFAULT
    };

    static AutonSelector &get_instance()
    {
        static AutonSelector instance;
        return instance;
    }

    void set_auton_routine(AutonRoutine routine)
    {
        selected_routine = routine;
    }

    void run_selected_routine()
    {
        switch (selected_routine)
        {
        case AutonRoutine::RED_MOGO:
            red_mogo();
            break;
        case AutonRoutine::BLUE_MOGO:
            blue_mogo();
            break;
        case AutonRoutine::RED_RING:
            red_ring();
            break;
        case AutonRoutine::BLUE_RING:
            blue_ring();
            break;
        case AutonRoutine::PROG_SKILLS:
            prog_skills();
            break;
        case AutonRoutine::DEFAULT:
            // Default routine or do nothing
            break;
        }
    }

private:
    AutonRoutine selected_routine = AutonRoutine::DEFAULT;

    AutonSelector() = default;
    ~AutonSelector() = default;
    AutonSelector(const AutonSelector &) = delete;
    AutonSelector &operator=(const AutonSelector &) = delete;
};

