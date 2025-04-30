#include "controller/modified_chassis.hpp"
#include <cmath>
#include "pros/misc.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"

namespace lemlib
{
    void ModifiedChassis::moveToPose(float x, float y, float theta, int timeout, MoveToPoseParams params, bool async)
    {
        // take the mutex
        this->requestMotionStart();
        // were all motions cancelled?
        if (!this->motionRunning)
            return;
        // if the function is async, run it in a new task
        if (async)
        {
            pros::Task task([&]()
                            { moveToPose(x, y, theta, timeout, params, false); });
            this->endMotion();
            pros::delay(10); // delay to give the task time to start
            return;
        }

        // reset PIDs and exit conditions
        lateralPID.reset();
        lateralLargeExit.reset();
        lateralSmallExit.reset();
        angularPID.reset();
        angularLargeExit.reset();
        angularSmallExit.reset();

        // calculate target pose in standard form
        Pose target(x, y, M_PI_2 - degToRad(theta));
        if (!params.forwards)
            target.theta = fmod(target.theta + M_PI, 2 * M_PI); // backwards movement

        // use global horizontalDrift is horizontalDrift is 0
        if (params.horizontalDrift == 0)
            params.horizontalDrift = drivetrain.horizontalDrift;

        // initialize vars used between iterations
        Pose lastPose = getPose();
        distTraveled = 0;
        Timer timer(timeout);
        bool close = false;
        bool lateralSettled = false;
        float prevLateralOut = 0; // previous lateral power
        float prevAngularOut = 0; // previous angular power
        const int compState = pros::competition::get_status();

        // Boomerang controller parameters
        const float dLead = params.lead;    // Carrot point lead parameter
        int dir = params.forwards ? 1 : -1; // Direction multiplier

        // main loop
        while (!timer.isDone() &&
               ((!lateralSettled || (!angularLargeExit.getExit() && !angularSmallExit.getExit())) || !close) &&
               this->motionRunning)
        {
            // update position
            const Pose pose = getPose(true, true);

            // update distance traveled
            distTraveled += pose.distance(lastPose);
            lastPose = pose;

            // calculate distance to the target point
            const float distTarget = pose.distance(target);

            // check if the robot is close enough to the target to start settling
            if (distTarget < 5 && close == false)
            {
                close = true;
                params.maxSpeed = fmax(fabs(prevLateralOut), 90);
            }

            // check if the lateral controller has settled
            if (lateralLargeExit.getExit() && lateralSmallExit.getExit())
                lateralSettled = true;

            // calculate the carrot point using boomerang controller approach
            Pose carrot(0,0,0);
            if (!close)
            {
                // Boomerang-style carrot point calculation
                float h = distTarget;
                carrot.x = target.x - (h * sin(target.theta) * dLead);
                carrot.y = target.y - (h * cos(target.theta) * dLead);
                carrot.theta = target.theta;
            }
            else
            {
                carrot = target; // Use target directly when settling
            }

            // Calculate adjusted target based on current heading
            float adjHeading = pose.theta;
            if (adjHeading > M_PI)
                adjHeading = -(2 * M_PI - adjHeading);
            float m = tan(adjHeading); // Slope of the robot's heading line

            // Determine which side of the line the robot is on
            float sideCheck;
            if (close)
            {
                // When close, calculate projection point on the line through robot with robot heading
                float tx = (m * (target.y - pose.y + pose.x * m + target.x / m)) / (m * m + 1);
                float ty = m * (tx - pose.x) + pose.y;

                // Determine side (-1 or 1)
                int side = pose.y < (-1 / m) * (pose.x - tx) + ty ? 1 : -1;
                if (side == 0)
                    side = -1;
                if (adjHeading < 0)
                    side = -side;
                dir = side * (params.forwards ? 1 : -1);
            }
            else
            {
                // When far, determine side based on carrot point
                int side = pose.y < (-1 / m) * (pose.x - carrot.x) + carrot.y ? 1 : -1;
                if (side == 0)
                    side = -1;
                if (adjHeading < 0)
                    side = -side;
                dir = side * (params.forwards ? 1 : -1);
            }

            // calculate error
            const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
            const float angularError =
                close ? angleError(adjustedRobotTheta, target.theta) : angleError(adjustedRobotTheta, pose.angle(carrot));

            // Calculate lateral error with boomerang-style modifications
            float lateralError = pose.distance(carrot);
            if (close)
                lateralError *= cos(angleError(pose.theta, pose.angle(carrot)));
            else
                lateralError *= sgn(cos(angleError(pose.theta, pose.angle(carrot))));

            // Apply direction multiplier from boomerang controller
            lateralError *= dir;

            // update exit conditions
            lateralSmallExit.update(lateralError);
            lateralLargeExit.update(lateralError);
            angularSmallExit.update(radToDeg(angularError));
            angularLargeExit.update(radToDeg(angularError));

            // get output from PIDs
            float lateralOut = lateralPID.update(lateralError);
            float angularOut = angularPID.update(radToDeg(angularError));

            // apply restrictions on angular speed
            angularOut = std::clamp(angularOut, -params.maxSpeed, params.maxSpeed);

            // apply restrictions on lateral speed
            lateralOut = std::clamp(lateralOut, -params.maxSpeed, params.maxSpeed);

            // constrain lateral output by max accel
            if (!close)
                lateralOut = slew(lateralOut, prevLateralOut, lateralSettings.slew);

            // constrain lateral output by the max speed it can travel at without slipping
            const float radius = 1 / fabs(getCurvature(pose, carrot));
            const float maxSlipSpeed(sqrt(params.horizontalDrift * radius * 9.8));
            lateralOut = std::clamp(lateralOut, -maxSlipSpeed, maxSlipSpeed);

            // Boomerang-style combined speed limiting
            if (std::abs(lateralOut) + std::abs(angularOut) > params.maxSpeed)
            {
                lateralOut = (params.maxSpeed - std::abs(angularOut)) * sgn(lateralOut);
            }

            // prevent moving in the wrong direction (keep existing logic)
            if (params.forwards && !close)
                lateralOut = std::fmax(lateralOut, 0);
            else if (!params.forwards && !close)
                lateralOut = std::fmin(lateralOut, 0);

            // constrain lateral output by the minimum speed    
            if (params.forwards && lateralOut < fabs(params.minSpeed) && lateralOut > 0)
                lateralOut = fabs(params.minSpeed);
            if (!params.forwards && -lateralOut < fabs(params.minSpeed) && lateralOut < 0)
                lateralOut = -fabs(params.minSpeed);

            // update previous output
            prevAngularOut = angularOut;
            prevLateralOut = lateralOut;

            // ratio the speeds to respect the max speed (keep existing logic)
            float leftPower = lateralOut + angularOut;
            float rightPower = lateralOut - angularOut;
            const float ratio = std::max(std::fabs(leftPower), std::fabs(rightPower)) / params.maxSpeed;
            if (ratio > 1)
            {
                leftPower /= ratio;
                rightPower /= ratio;
            }

            // move the drivetrain
            drivetrain.leftMotors->move(leftPower);
            drivetrain.rightMotors->move(rightPower);

            // delay to save resources
            pros::delay(10);
        }

        // stop the drivetrain
        drivetrain.leftMotors->move(0);
        drivetrain.rightMotors->move(0);
        // set distTraveled to -1 to indicate that the function has finished
        distTraveled = -1;
        this->endMotion();
    }
} // namespace lemlib