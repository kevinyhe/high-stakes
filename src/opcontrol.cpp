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

    if (!(intake.get_state() == mechanism::IntakeState::DEJAM || intake.get_state() == mechanism::IntakeState::WALL_STAKE))
    {
        if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
        {
            intake.set_state(mechanism::IntakeState::HOOK);
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
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A))
    {
        intake.disable_sort();
        intake.set_state(mechanism::IntakeState::DISABLED);
    }
}

void control_arm(pros::Controller &controller)
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))
    {
        if (arm.is_loading() || arm.is_primed())
        {
            intake.set_state(mechanism::IntakeState::WALL_STAKE);
            arm.set_state(mechanism::ArmState::NEUTRAL_STAKE);
        }
        else
        {
            arm.set_state(mechanism::ArmState::LOAD);
        }
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP))
    {
        intake.set_state(mechanism::IntakeState::WALL_STAKE);
        arm.set_state(mechanism::ArmState::PRIME);
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))
    {
        arm.set_state(mechanism::ArmState::IDLE);
    }

    // increase config::ARM_kP and config::ARM_kD
}

void control_clamp(pros::Controller controller)
{
    auto &clamp = mechanism::Clamp::get_instance();

    // if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN))
    // {
    //     clamp.set_autoclamp(!clamp.get_autoclamp());
    // }
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        clamp.toggle();
        
        if (!clamp.get_value())
        {
            controller.rumble("-");
            controller.print(0, 0, "CLAMP ON ");
        }
        else
        {
            controller.rumble(".");
            controller.print(0, 0, "CLAMP OFF");
        }
    }
}

void control_doinker(pros::Controller controller)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT))
    {
        doinker.toggle();
    }
}

void opcontrol()
{
    auto &clamp = mechanism::Clamp::get_instance();
    auto &intake = mechanism::Intake::get_instance();

    clamp.set_autoclamp(false);

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