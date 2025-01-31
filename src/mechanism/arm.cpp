#include "arm.hpp"
#include "pros/misc.h"


mechanism::Arm::Arm(std::shared_ptr<pros::Motor> motors, std::shared_ptr<pros::Rotation> arm_rotation_sensor, std::shared_ptr<mechanism::Intake> intake, std::shared_ptr<PID> arm_pid, ArmTargetConfig target_config, double kG)
    : motors(motors),
      arm_rotation_sensor(arm_rotation_sensor),
      intake(intake),
      arm_pid(arm_pid),
      target_config(target_config),
      kG(kG)
{
    if (!motors->is_installed())
    {
        // ERROR_TEXT("Arm motor(s) unplugged!");
    }
    if (!arm_rotation_sensor->is_installed())
    {
        // ERROR("Arm rotation sensor is not detected on port %d!", arm_rotation_sensor->get_port());
    }

    start_task();
}

void mechanism::Arm::start_task()
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
            mutex.unlock();

            double current_angle = ((arm_rotation_sensor->get_position() / 100.0) - 360) / 5.0; // divide by 100 to convert centidegrees to degrees

            double target_angle = INFINITY;
                
            switch (state)
            {
            case ArmState::LOAD:
            {
                target_angle = target_config.load;
            }
            break;
            case ArmState::ALLIANCE_STAKE:
            {
                target_angle = target_config.alliance_stake;
            }
            break;
            case ArmState::NEUTRAL_STAKE:
            {
                target_angle = target_config.neutral_stake;
            }
            break;
            case ArmState::LADDER_TOUCH:
            {
                target_angle = target_config.ladder_touch;
            }
            break;
            default: break;
            }

            
            if (is_loading() && intake->get_state() == IntakeState::WALL_STAKE)
            {
                motors->move(-5);
            }
            else if (target_angle != INFINITY)
            {
                double output = arm_pid->calculate(current_angle, target_angle);
                // std::cout << current_angle << ", " << target_angle << ", " << output << std::endl;

                // if (arm_limit->arm_limit->get_value() == true)
                // {
                //     output = std::max(output, 0.0);
                // }

                double real_theta = -current_angle / 5.0 * M_PI / 180.0 + M_PI / 2.0;

                if (output > -50)
                {
                    motors->move(output + kG * cos(real_theta));
                }
                else
                {
                    motors->move(-50);
                }
            }

            pros::delay(20);
        } });
}

void mechanism::Arm::stop_task()
{
    mutex.lock();
    task_on_flag = false;
    mutex.unlock();
}

void mechanism::Arm::set_state(mechanism::ArmState state)
{
    mutex.lock();
    if (state == ArmState::DISABLED)
    {
        motors->move(0);
    }

    this->state = state;
    mutex.unlock();
}

mechanism::ArmState mechanism::Arm::get_state()
{
    mutex.lock();
    auto temp = state;
    mutex.unlock();

    return temp;
}

bool mechanism::Arm::is_loading()
{
    double current = ((arm_rotation_sensor->get_position() / 100.0) - 360) / 5.0;

    if (current > 180.0)
    {
        current = 0.0;
    }

    return std::abs(current - target_config.load) < 3.0;
}

// void mechanism::Arm::move_manual(double voltage)
// {
//     mutex.lock();
//     state = ArmState::DISABLED;

//     double current_angle = ((arm_rotation_sensor->get_position() / 100.0) - 360) / 5.0;

//     double real_theta = -current_angle * M_PI / 180.0;

//     motors->move(voltage + kG * cos(real_theta));
//     mutex.unlock();
// }