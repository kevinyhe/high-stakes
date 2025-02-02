#include "main.h"
#include "config.hpp"

// r1 for fwd
// r2 for reverse TODO: remove
// l2 to load

// y for neutral
// x for alliance

void control_intake(pros::Controller &controller)
{
    auto &intake = mechanism::Intake::get_instance();
    auto &arm = mechanism::Arm::get_instance();

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intake.set_state(mechanism::IntakeState::HOOK);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.set_state(mechanism::IntakeState::REVERSE);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        intake.set_state(mechanism::IntakeState::WALL_STAKE);

        if (arm.get_state() != mechanism::ArmState::LOAD)
        {
            arm.set_state(mechanism::ArmState::LOAD);
        }
    }
    else
    {
        intake.set_state(mechanism::IntakeState::DISABLED);
    }
}

void control_arm(pros::Controller &controller)
{
    auto &arm = mechanism::Arm::get_instance();

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        if (arm.is_loading())
        {
            arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
        }
        else
        {
            arm.set_state(mechanism::ArmState::LOAD);
        }
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        arm.set_state(mechanism::ArmState::ALLIANCE_STAKE);
    }
}

void opcontrol()
{
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true)
    {
        int fwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        chassis->tank(fwd, turn);

        control_arm(controller);
        control_intake(controller);

        pros::delay(20);
    }
}