#include "arm.hpp"
#include "config.hpp"
#include "main.h"

namespace mechanism
{
    std::unique_ptr<Arm> Arm::instance = nullptr;

    Arm::Arm(std::shared_ptr<pros::Motor> motors,
             std::shared_ptr<pros::Rotation> arm_rotation_sensor,
             std::shared_ptr<PID> arm_pid,
             ArmTargetConfig target_config,
             double kG)
        : motors(motors),
          arm_rotation_sensor(arm_rotation_sensor),
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

    Arm::~Arm()
    {
        stop_task();
    }

    void Arm::start_task()
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
                if (task_on_flag == false || pros::competition::is_disabled()) {
                    mutex.unlock();
                    break;
                }
                mutex.unlock();

                double current_angle = ((arm_rotation_sensor->get_position() / 100.0) - 360) / 5.0;

                double target_angle = INFINITY;
                switch (state) {
                    case ArmState::LOAD: target_angle = target_config.load; break;
                    case ArmState::ALLIANCE_STAKE: target_angle = target_config.alliance_stake; break;
                    case ArmState::NEUTRAL_STAKE: target_angle = target_config.neutral_stake; break;
                    case ArmState::LADDER_TOUCH: target_angle = target_config.ladder_touch; break;
                    default: break;
                }

                // Access Intake singleton directly
                auto& intake = Intake::get_instance();

                if (is_loading() && intake.get_state() == IntakeState::WALL_STAKE) {
                    motors->move(5); // holding power
                }
                else if (target_angle != INFINITY) {
                    double output = arm_pid->calculate(current_angle, target_angle);
                    double real_theta = -current_angle * config::ARM_GEAR_RATIO * M_PI / 180.0 + M_PI / 2.0; // of motor

                    if (output > 50) {
                        motors->move(output + kG * cos(real_theta));
                    }
                    else {
                        motors->move(50);
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

} // namespace mechanism