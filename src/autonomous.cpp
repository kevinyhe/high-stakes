#include "main.h"
#include "config.hpp"
#include "autonomous.hpp"

void autonomous()
{
    red_mogo();
    // get the potentiometer value
    // int pot_value = potentiometer.get_value();
    // // depending on which auton is selected, run the corresponding function
    // // get one of 8 ranges from potentiometer
    // int pot_range = std::floor(pot_value / 45);

    // switch (pot_range)
    // {   
    //     case 0:
    //         controller.print(0, 0, "Red Mogo");
    //         red_mogo();
    //         break;
    //     case 1:
    //         controller.print(0, 0, "Blue Mogo");
    //         blue_mogo();
    //         break;
    //     case 2:
    //         controller.print(0, 0, "Red Ring");
    //         red_ring();
    //         break;
    //     case 3:
    //         controller.print(0, 0, "Blue Ring");
    //         blue_ring();
    //         break;
    //     case 4:
    //         controller.print(0, 0, "Prog Skills");
    //         prog_skills();
    //         break;
    //     case 5:
    //         controller.print(0, 0, "Default");
    //         break;
    //     default:
    //         break;
    //     }
}