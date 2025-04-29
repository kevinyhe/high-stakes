#pragma once

#include "lemlib/chassis/chassis.hpp"
#include "lemlib/pose.hpp"
#include "lemlib/pid.hpp"
#include "lemlib/timer.hpp"
#include <memory>

namespace lemlib
{
    /**
     * @brief ModifiedChassis class with advanced control laws
     */
    class ModifiedChassis : public Chassis
    {
    public:
        /**
         * @brief Construct a new Modified Chassis
         *
         * @param drivetrain drivetrain to be used for the chassis
         * @param linearSettings settings for the lateral controller
         * @param angularSettings settings for the angular controller
         * @param sensors sensors to be used for odometry
         * @param lateralGain lateral correction gain coefficient
         * @param throttleCurve curve applied to throttle input during driver control
         * @param steerCurve curve applied to steer input during driver control
         */
        ModifiedChassis(
            Drivetrain drivetrain,
            ControllerSettings linearSettings,
            ControllerSettings angularSettings,
            OdomSensors sensors,
            float lateralGain = 3.0,
            DriveCurve *throttleCurve = &defaultDriveCurve,
            DriveCurve *steerCurve = &defaultDriveCurve)
            : Chassis(drivetrain, linearSettings, angularSettings, sensors,
                      throttleCurve, steerCurve),
              k_lat(lateralGain) {}

        /**
         * @brief Move the chassis to a target pose using advanced control law with original LemLib parameters
         *
         * @param x x coordinate
         * @param y y coordinate
         * @param theta target heading in degrees
         * @param timeout maximum time for the movement in milliseconds
         * @param params movement parameters
         * @param async whether to run the movement asynchronously
         */
        void moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params = {}, bool async = true);

        void driveDistancePID(float distance, float maxSpeed, int timeout);

        /**
         * @brief Execute one iteration of the control loop
         * Implements the advanced control law
         */
        void step();

        /**
         * @brief Check if the chassis has reached the target
         *
         * @return true if the chassis is at the target pose within tolerances
         * @return false otherwise
         */
        bool settled();

        /**
         * @brief Set the lateral gain coefficient
         *
         * @param gain new gain value
         */
        void setLateralGain(float gain) { k_lat = gain; }

        /**
         * @brief Get the lateral gain coefficient
         *
         * @return float current gain value
         */
        float getLateralGain() const { return k_lat; }

    private:
        Pose targetPose = {0, 0, 0};
        float k_lat = 3.0;
        bool isReversed = false;
        int timeoutMs = 0;
        Timer *timer = nullptr;
        MoveToPoseParams currentParams = {};
    };
} // namespace lemlib