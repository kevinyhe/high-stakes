#pragma once

#include <vector>
#include <sstream>
#include <algorithm>
#include <cmath>
#include "pros/imu.hpp"
#include "pros/rtos.hpp"

// Base class for heading sources.
class HeadingSource
{
public:
    virtual ~HeadingSource() = default;
    // Returns the current heading, or NaN if sensor is malfunctioning.
    virtual double getHeading() const = 0;
    // Calibrates the sensor.
    virtual bool calibrate() = 0;
    // Returns true if sensor is done calibrating.
    virtual bool isDoneCalibrating() const = 0;

    void disable() { _isEnabled = false; }
    void enable() { _isEnabled = true; }
    bool isEnabled() const { return _isEnabled; }

private:
    bool _isEnabled = true;
};

// Implementation of HeadingSource using a pros::IMU.
class IMUHeadingSource : public HeadingSource
{
public:
    IMUHeadingSource(pros::IMU *imu, const float _gain = 1);
    double getHeading() const override;
    bool calibrate() override;
    bool isDoneCalibrating() const override;

protected:
    float gain;

private:
    pros::IMU *imu;
};

// A mock IMU class that aggregates multiple HeadingSource objects.
// Inherits from pros::IMU so it can be used in place of a normal IMU.
class MockIMU : public pros::IMU
{
public:
    explicit MockIMU(std::vector<HeadingSource *> sources);

    // Overrides for the pros::IMU interface.
    double get_heading() const override;
    double get_rotation() const override;
    int32_t reset(bool idk) const override;

    // Calibrate and check calibration status.
    bool calibrate();
    bool is_calibrating() const override;

private:
    std::vector<HeadingSource *> sources;
};
