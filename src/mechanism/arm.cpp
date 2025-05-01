#include "arm.hpp"
#include "config.hpp"
#include "main.h"

namespace mechanism
{
    std::unique_ptr<Arm> Arm::instance = nullptr;

    Arm::Arm(std::shared_ptr<pros::MotorGroup> motors,
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

        arm_rotation_sensor->set_data_rate(5);
        arm_rotation_sensor->set_position(0);

        task_on_flag = true;
        mutex.unlock();

        // arm_rotation_sensor.reset();

        task = pros::Task([this]()
                          {
            while (true)
            {
                mutex.lock();
                if (task_on_flag == false)
                {
                    break;
                }
                mutex.unlock();

                pros::lcd::print(3, "rot: %d", arm_rotation_sensor->get_position());

                double current_angle = (arm_rotation_sensor->get_position() / 100.0);

                double target_angle = INFINITY;
                switch (state) {
                    case ArmState::LOAD: target_angle = target_config.load;
                        break;
                    case ArmState::IDLE:
                        target_angle = target_config.idle;
                    break;
                    case ArmState::NEUTRAL_STAKE:
                        target_angle = target_config.neutral_stake;
                    break;
                    case ArmState::PRIME: target_angle = target_config.prime;
                     break;
                    case ArmState::ALLIANCE_STAKE:
                        target_angle = target_config.alliance_stake;
                    default:
                    break;
                }

                // Access Intake singleton directly
                auto& intake = Intake::get_instance();
                // relieve stress on motors for idle position
                if (target_angle == target_config.idle && current_angle < target_config.idle + 50) {
                    motors->move(0);
                }
                else if (target_angle == target_config.neutral_stake && current_angle < target_config.neutral_stake - 50) {
                    motors->move(127);
                }
                else if (target_angle != INFINITY) {
                    double output = arm_pid->calculate(current_angle, target_angle);
                    // if (output < 50) {
                        motors->move(output + (kG * cos((current_angle-50) * M_PI / 180.0)));
                        pros::lcd::print(5, "output: %f", kG * cos((current_angle-50) * M_PI / 180.0));
                        // motors->move(output);
                        // } else {
                        // motors->move(127);
                        // }
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
        double current = (arm_rotation_sensor->get_position() / 100.0);

        return std::abs(current - target_config.load) < 4;
    }

    bool mechanism::Arm::is_primed()
    {
        double current = (arm_rotation_sensor->get_position() / 100.0);

        return std::abs(current - target_config.prime) < 15;
    }
    void mechanism::Arm::set_rotation_value(double v) {
        mutex.lock();

        arm_rotation_sensor->set_position(v * 100.0);
        mutex.unlock();
    }


} // namespace mechanism