#include "intake.hpp"
#include "pros/misc.h"

#define LOGGER "intake.cpp"

mechanism::Intake::Intake(std::shared_ptr<pros::MotorGroup> motors)
    : motors(motors)
{
    start_task();
}

void mechanism::Intake::start_task()
{
    mutex.lock();
    if (task_on_flag)
    {
        mutex.unlock();
        return;
    }

    task_on_flag = true;
    mutex.unlock();

    task = pros::Task([this]()
                      {
        
        while (true)
        {
            mutex.lock();
            if (task_on_flag == false || ((pros::c::competition_get_status() & COMPETITION_DISABLED) != 0))
            {
                break;
            }

            if (this->state != IntakeState::DISABLED && this->state != IntakeState::DEJAM && std::abs(motors->get_actual_velocity()) < 300 && std::abs(motors->get_voltage()) > 0)
            {
                this->state = IntakeState::DEJAM;
            }

            IntakeState state = this->state;

            mutex.unlock();
            
            switch (state)
            {
            case IntakeState::HOOD: {
                motors->move(127);
            } break;
            case IntakeState::WALL_STAKE: {
                motors->move(127);
            } break;
            case IntakeState::REVERSE: {
                motors->move(-127);
            } break;
            case IntakeState::DEJAM: {
                motors->move(-127);
            } break;
            case IntakeState::DISABLED: {
                motors->move(0);
            } break;
            }

            pros::delay(20);
        } });
}

void mechanism::Intake::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Intake::set_state(mechanism::IntakeState state)
{
    mutex.lock();
    if (state == IntakeState::DISABLED)
    {
        motors->move(0);
    }

    this->state = state;
    mutex.unlock();
}

mechanism::IntakeState mechanism::Intake::get_state()
{
    mutex.lock();
    auto temp = state;
    mutex.unlock();

    return temp;
}