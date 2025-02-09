#include "main.h"
#include "base_odom.hpp"
#include <random>

    struct MCLSensorParams
{
    int port;
    double x_offset;
    double y_offset;
    double theta_offset;
};

struct MCLConfig
{
    // The number of particles to use for the particle filter
    int particle_count;
    // What percent of the particles should be uniformly random; the rest are
    // based on the previous probability distribution
    double uniform_random_percent;
    // The standard deviation of the tracking wheel measurements
    double tracker_odom_sd;
};

class MCLOdom : public virtual IOdometry
{
private:
    struct MCLSensor
    {
        std::shared_ptr<pros::Distance> sensor;
        double x_offset;
        double y_offset;
        double theta_offset;
    };

    struct MCLParticle
    {
        double x;
        double y;
        double weight;
    };

    std::shared_ptr<lemlib::Chassis> tracker_odom;
    double last_x;
    double last_y;

    std::vector<MCLSensor> sensors;

    std::vector<MCLParticle> particles;

    pros::Mutex position_mutex;
    double predicted_x = 0;
    double predicted_y = 0;

    std::mt19937 rng; // pseudorandom number generator, mersenne twister

    int particle_count;
    double uniform_random_percent;
    double tracker_odom_sd;

    pros::Task task = pros::Task([]
                                 { return; });

    static double sensor_sd(double distance);
    static double get_particle_chance(double x, double y, MCLSensor sensor,
                                      double theta);

    static double gaussian_distribution(double x, double mean, double sd);

    double random();
    double gaussian_random(double mean, double sd);

    void motion_update();
    void resample();
    void sensor_update();

public:
    MCLOdom(MCLConfig config, std::shared_ptr<lemlib::Chassis> tracker_odom,
            std::vector<MCLSensorParams>);
    ~MCLOdom();

    void initialize(double initial_x, double initial_y, Angle initial_theta);

    double get_x();
    double get_y();
    Vec get_position();
    Pose get_pose();

    Angle get_rotation();
    void set_rotation(Angle rotation);

    void print_telemetry();

    /**
     * @brief Start the odometry background task
     */
    void start_task(double initial_x, double initial_y, Angle initial_theta);
    /**
     * @brief Stop the odometry background task
     */
    void stop_task();
};
