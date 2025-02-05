#include "intake.hpp"
#include "config.hpp"

namespace mechanism
{
    std::unique_ptr<Intake> Intake::instance = nullptr;
    std::once_flag Intake::init_flag;

    Intake::Intake(std::shared_ptr<pros::MotorGroup> motors)
        : motors(motors)
    {
        start_task();
    }

    Intake::~Intake()
    {
        stop_task();
    }

    void Intake::start_task()
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
            auto initial_comp_status = pros::c::competition_get_status();
        
            while (pros::c::competition_get_status() == initial_comp_status)
            {
                mutex.lock();
                if (task_on_flag == false)
                {
                    mutex.unlock();
                    break;
                }

                // Check if the motors are at 0 velocity
                if (std::abs(motors->get_actual_velocity(1)) < 20 && this->state != IntakeState::DISABLED && this->state != IntakeState::DEJAM && this->state != IntakeState::WALL_STAKE)
                {
                    // If the motors are at 0 velocity, start or update the timer
                    if (zero_velocity_start_time == 0)
                    {
                        zero_velocity_start_time = pros::millis();
                    }
                    else if (pros::millis() - zero_velocity_start_time > 400)
                    {
                        // If the motors have been at 0 velocity for more than 500 ms, activate dejam state
                        this->dejam_start_time = pros::millis();
                        this->pre_dejam_state = this->state;
                        this->state = IntakeState::DEJAM;
                    }
                }
                else
                {
                    // If the motors are not at 0 velocity, reset the timer
                    zero_velocity_start_time = 0;
                }

                // If jammed for a certain amount of time return to initial state
                if (this->state == IntakeState::DEJAM && pros::millis() - this->dejam_start_time > 500)
                {
                    this->state = this->pre_dejam_state;
                }

                IntakeState current_state = this->state;
                mutex.unlock();
                
                switch (current_state) {
                    case IntakeState::HOOK:
                        motors->move(127);
                        pros::lcd::print(3, "hbook");
                        break;
                    case IntakeState::WALL_STAKE:
                        motors->move(127);
                        pros::lcd::print(3, "stake");
                        break;
                    case IntakeState::REVERSE:
                        motors->move(-127);
                        pros::lcd::print(3, "reverse");
                        break;
                    case IntakeState::DEJAM:
                        motors->move(-127);
                        pros::lcd::print(3, "dejam");
                        break;
                    case IntakeState::DISABLED:
                        motors->move(0);
                        pros::lcd::print(3, "disabled");
                        break;
                }

                pros::delay(20);
            } });
    }

    void Intake::stop_task()
    {
        mutex.lock();
        task_on_flag = false;
        mutex.unlock();
    }

    void Intake::set_state(IntakeState new_state)
    {
        mutex.lock();
        if (new_state == IntakeState::DISABLED)
        {
            motors->move(0);
        }
        state = new_state;
        mutex.unlock();
    }

    IntakeState Intake::get_state()
    {
        mutex.lock();
        auto temp = state;
        mutex.unlock();
        return temp;
    }
} // namespace mechanism