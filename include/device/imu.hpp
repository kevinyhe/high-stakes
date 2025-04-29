#pragma once

#include "main.h"
#include "pros/imu.hpp"
#include "filter/Kalman.h"
#include <cmath>

class MockIMU : public pros::Imu
{
public:
    MockIMU(int port, double gain = 1.0)
        : pros::Imu(port), imu_gain(gain)
    {
        // Initialize Kalman filter with better parameters from Arduino implementation
        kalmanFilter = new Kalman();
        
        // These parameters match the Arduino implementation
        kalmanFilter->setQangle(0.001f);
        kalmanFilter->setQbias(0.003f);
        kalmanFilter->setRmeasure(0.03f);
        
        // Set proper covariance matrix initialization
        // (The Arduino implementation properly initializes this in the Kalman class)
        kalmanFilter->resetFilter();
        
        // Initialize state variables
        lastTime = pros::millis();
        gyroAngle = 0;
        compAngle = 0;
        kalAngle = 0;
        
        // Initialize with current values (like in Arduino setup)
        double initial_rotation = pros::Imu::get_rotation();
        kalmanFilter->setAngle(initial_rotation); // Set starting angle
        gyroAngle = initial_rotation;
        compAngle = initial_rotation;
        kalAngle = initial_rotation;
        lastAngle = initial_rotation;
    }
    
    ~MockIMU() {
        delete kalmanFilter;
    }

    double get_rotation() const override
    {
        // Get raw rotation from IMU
        double raw = pros::Imu::get_rotation();
        if (raw == PROS_ERR_F)
            return NAN;

        // Calculate delta time (similar to Arduino implementation)
        uint32_t currentTime = pros::millis();
        float dt = (currentTime - lastTime) / 1000.0f; // Convert to seconds
        if (dt <= 0)
            dt = 0.001f; // Prevent division by zero

        // Get gyro rate from IMU
        double gyroRate;
        pros::imu_gyro_s_t gyro = pros::Imu::get_gyro_rate();
        if (gyro.z != PROS_ERR_F)
        {
            gyroRate = gyro.z; // Use Z-axis rotation rate
        }
        else
        {
            // Fallback: calculate from angle difference
            gyroRate = (raw - lastAngle) / dt;
        }

        // Calculate all three angles like in Arduino implementation

        // 1. Pure gyro integration (similar to gyroXangle in Arduino)
        gyroAngle += gyroRate * dt;

        // 2. Complementary filter (similar to compAngleX in Arduino)
        // Using 0.93/0.07 weights as in the Arduino example rather than 0.98/0.02
        compAngle = 0.93 * (compAngle + gyroRate * dt) + 0.07 * raw;

        // Check for large angle jumps (like the Arduino transition fix)
        if ((raw < -90 && kalAngle > 90) || (raw > 90 && kalAngle < -90)) {
            // If we jump across the discontinuity, reset the filter
            kalmanFilter->setAngle(raw);
            kalAngle = raw;
            compAngle = raw;
            gyroAngle = raw;
        } else {
            // 3. Kalman filter (best but most complex)
            kalAngle = kalmanFilter->getAngle(raw, gyroRate, dt);
        }

        // Reset gyro angle when it has drifted too much (like in Arduino)
        if (gyroAngle < -180 || gyroAngle > 180)
            gyroAngle = kalAngle;

        // Update history
        lastTime = currentTime;
        lastAngle = raw;

        // Return Kalman filtered angle with gain
        return kalAngle * imu_gain;
    }
    
    // Get access to all filtering methods (like in Arduino)
    double get_gyro_angle() const {
        get_rotation(); // Update all values
        return gyroAngle * imu_gain;
    }
    
    double get_comp_angle() const {
        get_rotation(); // Update all values
        return compAngle * imu_gain;
    }
    
    double get_kalman_angle() const {
        return get_rotation(); // Already returns kalAngle
    }
    
    // Add methods to tune parameters (based on Arduino implementation)
    void set_filter_params(float q_angle, float q_bias, float r_measure) {
        kalmanFilter->setQangle(q_angle);
        kalmanFilter->setQbias(q_bias);
        kalmanFilter->setRmeasure(r_measure);
    }
    
    // Method to diagnose and compare filters like in Arduino loop
    void print_diagnostics() const {
        double raw = pros::Imu::get_rotation();
        double gyro = get_gyro_angle();
        double comp = get_comp_angle();
        double kalman = get_kalman_angle();
        
        pros::lcd::print(0, "Raw: %.2f", raw);
        pros::lcd::print(1, "Gyro: %.2f", gyro);
        pros::lcd::print(2, "Comp: %.2f", comp);
        pros::lcd::print(3, "Kalman: %.2f", kalman);
    }

private:
    double imu_gain;
    Kalman *kalmanFilter;

    // Timing variables
    mutable uint32_t lastTime;
    mutable double lastAngle;

    // Different filtering methods (like Arduino implementation)
    mutable double gyroAngle; // Pure gyro integration (prone to drift)
    mutable double compAngle; // Complementary filter (simple fusion)
    mutable double kalAngle;  // Kalman filter (best accuracy)
};