#include "autonomous.hpp"
#include "config.hpp"
#include "main.h"

void red_mogo()
{
    auto &arm = mechanism::Arm::get_instance();
    auto &intake = mechanism::Intake::get_instance();
    auto &clamp = mechanism::Clamp::get_instance();

    intake.enable_sort(mechanism::Intake::RingColours::BLUE);

}