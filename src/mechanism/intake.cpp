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
            while (true) {
                mutex.lock();
                if (task_on_flag == false || ((pros::c::competition_get_status() & COMPETITION_DISABLED) != 0)) {
                    mutex.unlock();
                    break;
                }

                if (this->state != IntakeState::DISABLED && 
                    this->state != IntakeState::DEJAM && 
                    std::abs(motors->get_actual_velocity()) < 300) {
                    this->state = IntakeState::DEJAM;
                }

                IntakeState current_state = this->state;
                mutex.unlock();
                
                switch (current_state) {
                    case IntakeState::HOOK:
                    case IntakeState::WALL_STAKE:
                        motors->move(127);
                        break;
                    case IntakeState::REVERSE:
                    case IntakeState::DEJAM:
                        motors->move(-127);
                        break;
                    case IntakeState::DISABLED:
                        motors->move(0);
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