#include "main.h"
#include "mechanism/arm.hpp"
#include "mechanism/intake.hpp"
#include "config.h"

bool intake_on = true;
bool arm_on = false;

// r1 for fwd
// r2 for reverse TODO: remove
// l2 to load

void control_intake(pros::Controller controller, std::shared_ptr<mechanism::Intake> intake, std::shared_ptr<mechanism::Arm> arm)
{
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1))
    {
        intake->set_state(mechanism::IntakeState::HOOK);
    }

    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2))
    {
        intake->set_state(mechanism::IntakeState::REVERSE);
    }
    else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2))
    {
        intake->set_state(mechanism::IntakeState::WALL_STAKE);

        if (arm->get_state() != mechanism::ArmState::LOAD)
        {
            arm->set_state(mechanism::ArmState::LOAD);
        }
    }
    else
    {
        intake->set_state(mechanism::IntakeState::DISABLED);
    }
}

// y for neutral
// x for alliance
void control_arm(pros::Controller controller, std::shared_ptr<mechanism::Arm> arm)
{
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y))
    {
        if (arm->is_loading())
        {
            arm->set_state(mechanism::ArmState::NEUTRAL_STAKE);
        }
        else
        {
            arm->set_state(mechanism::ArmState::LOAD);
        }
    }
    else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X))
    {
        arm->set_state(mechanism::ArmState::ALLIANCE_STAKE);
    }
    // else if (arm_manual)
    // {
    //     arm->move_manual(0);
    //     arm_manual = false;
    // }
}

void opcontrol()
{
    while (true)
    {
        int fwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

        // chassis.tank(fwd, turn);

        control_arm(controller, arm);
        control_intake(controller, intake, arm);

        pros::delay(20);
    }
}