#pragma once

#include <memory>

#include "../math/angle.hpp"
#include "../math/pose.hpp"
#include "../math/vector.hpp"

#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <random>


class IOdometry
{
public:
    /**
     * @brief Initialize the odometry algorithm. If the algorithm runs in a
     * seperate task, it starts the task
     *
     * @param x The initial X position
     * @param y The inital Y position
     * @param theta The initial heading
     */
    virtual void initialize(double x, double y, Angle theta) = 0;

    /**
     * @brief Get the x position of the robot
     */
    virtual double get_x() = 0;
    /**
     * @brief Get the y position of the robot
     */
    virtual double get_y() = 0;
    /**
     * @brief Get the position of the robot
     */
    virtual Vec get_position() = 0;
    /**
     * @brief Get the pose of the robot
     */
    virtual Pose get_pose() = 0;

    /**
     * @brief Get the rotation of the robot
     */
    virtual Angle get_rotation() = 0;
    /**
     * @brief Set the rotation of the robot
     *
     * @param rotation The new rotation of the robot
     */
    virtual void set_rotation(Angle rotation) = 0;
};