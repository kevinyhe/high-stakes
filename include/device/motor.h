// bool MotorGroup::is_installed()
// {
//     bool installed = true;

//     for (int i = 0; i < (int)motors.size(); i++)
//     {
//         if (!motors[i]->is_installed())
//         {
//             installed = false;
//             break;
//         }
//     }

//     return installed;
// }

// std::vector<int> MotorGroup::get_uninstalled_motors()
// {
//     std::vector<int> uninstalled = std::vector<int>();

//     for (int i = 0; i < (int)motors.size(); i++)
//     {
//         if (!motors[i]->is_installed())
//         {
//             uninstalled.push_back(motors[i]->get_port());
//         }
//     }

//     return uninstalled;
// }