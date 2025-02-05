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
        if (!(intake.get_state() == mechanism::IntakeState::DEJAM))
        {
            intake.set_state(mechanism::IntakeState::HOOK);
        }
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake.set_state(mechanism::IntakeState::REVERSE);
    }
    else
    {
        intake.set_state(mechanism::IntakeState::DISABLED);
    }
}

void control_arm(pros::Controller &controller)
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        if (arm.is_loading())
        {
            arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
            // intake.set_state(mechanism::IntakeState::DISABLED);
            intake.set_state(mechanism::IntakeState::WALL_STAKE);
        }
        else
        {
            arm.set_state(mechanism::ArmState::LOAD);
        }
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        arm.set_state(mechanism::ArmState::IDLE);
    }

    // increase config::ARM_kP and config::ARM_kD
}

void control_clamp(pros::Controller controller)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        clamp.toggle();

        if (!clamp.get_value())
        {
            controller.print(0, 0, "CLAMP ON");
        }
        else
        {
            controller.print(0, 0, "CLAMP OFF");
        }
    }
}

void control_doinker(pros::Controller controller)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B))
    {
        doinker.toggle();
    }
}

void opcontrol()
{
    pros::Controller controller(pros::E_CONTROLLER_MASTER);

    while (true)
    {
        int fwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis->arcade(fwd, turn);

        control_arm(controller);
        control_intake(controller);
        control_clamp(controller);
        control_doinker(controller);

        pros::delay(20);
    }
}