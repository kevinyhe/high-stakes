#include "config.hpp"
#include "autonomous.hpp"
#include "main.h"

void prog_skills()
{
    chassis->setPose(-65.5, 0, 90);

    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    chassis->moveToPose(-48, 0, 90, 10000);
    //TODO: test starting pose to ensure on line
}
