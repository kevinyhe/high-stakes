// #include "main.h"
// #include "config.hpp"
// #include "mcl/mcl.hpp"

// #define FIELD_WIDTH 144 // Field width in inches (assumes a square field)

// // Constructor for the Monte Carlo Localization class
// MCLOdom::MCLOdom(MCLConfig config,
//                         std::shared_ptr<lemlib::Chassis> tracker_odom,
//                         std::vector<MCLSensorParams> sensor_params)
//     : tracker_odom(tracker_odom),
//       uniform_random_percent(config.uniform_random_percent),
//       tracker_odom_sd(config.tracker_odom_sd) {
//   // Initialize random seed based on battery state and system time
//   // more efficient state space exploration, won't get stuck in local minima
//   int system_time = pros::micros();
//   double battery_current = pros::c::battery_get_current();
//   double battery_voltage = pros::c::battery_get_voltage();

//   int seed = int(battery_voltage + battery_current * 100) + system_time;
//   rng.seed(seed); // Seed the random number generator

//   // Initialize distance sensors with given parameters
//   for (MCLSensorParams params : sensor_params) {
//     sensors.push_back(MCLSensor{
//         .sensor =
//             std::make_shared<pros::Distance>(params.port), // Sensor object
//         .x_offset = params.x_offset,                       // Sensor X offset
//         .y_offset = params.y_offset,                       // Sensor Y offset
//         .theta_offset = params.theta_offset // Sensor angle offset
//     });
//   }
// }

// // Destructor for the MCL class
// MCLOdom::~MCLOdom() { stop_task(); }

// // Initialize particles and the task for running MCL
// void MCLOdom::initialize(double initial_x, double initial_y,
//                                 Angle initial_theta) {
//   start_task(initial_x, initial_y, initial_theta);
// }

// // Sensor standard deviation based on the distance measured
// // Accounts for 5% variance above 200mm or 0.59 inches minimum
// double MCLOdom::sensor_sd(double distance) {
//   double variance = std::max(distance * 0.05, 0.590551);
//   return variance / 3; // Convert variance to standard deviation
// }

// // Compute the weight (likelihood) of a particle based on a sensor's reading
// double MCLOdom::get_particle_chance(double x, double y, MCLSensor sensor,
//                                            double theta) {
//   // Convert sensor distance from mm to inches
//   double distance = sensor.sensor->get_distance() * 0.0393701;

//   // Compute offsets for the sensor's position relative to the particle
//   double x_offset =
//       sensor.y_offset * cos(theta) + sensor.x_offset * cos(theta - M_PI / 2.0);
//   double y_offset =
//       sensor.y_offset * sin(theta) + sensor.x_offset * sin(theta - M_PI / 2.0);

//   // Predict the expected distance from the particle to the wall in X and Y
//   // rotate(x, y) is rotating if you have a precomputed sine and cosine

//   double x_predict = (cos(theta) > 0 ? FIELD_WIDTH - distance * cos(theta)
//                                          : distance * std::abs(cos(theta))) +
//                          x_offset;
//   double y_predict = (sin(theta) > 0 ? FIELD_WIDTH - distance * sin(theta)
//                                      : distance * std::abs(sin(theta))) +
//                      y_offset;

//   // Return the Gaussian distribution value for the predicted and actual
//   // distances
//   return std::max(gaussian_distribution(x, x_predict, sensor_sd(distance)),
//                   gaussian_distribution(y, y_predict, sensor_sd(distance)));
// }

// // Generate a random double in [0, 1)
// double MCLOdom::random() { return (double)rng() / (double)rng.max(); }

// // Generate a random number from a Gaussian distribution
// double MCLOdom::gaussian_random(double mean, double sd) {
//   double u = 1 - random();
//   double v = random();
//   double z = sqrt(-2.0 * log(u)) * cos(2.0 * M_PI * v); // Box-Muller transform
//   return z * sd + mean;                                 // Scale and shift
// }

// // Compute the Gaussian distribution value for a given x
// double MCLOdom::gaussian_distribution(double x, double mean, double sd) {
//   double EPSILON = 1e-19; // Minimum probability to avoid zero
//   // if this doesn't work 0.0000000000000000001
//   return std::max(pow(M_E, -1.0 / 2.0 * pow((x - mean) / sd, 2)) /
//                       (sd * sqrt(2 * M_PI)),
//                   EPSILON);
// }

// // Update particles based on odometry (motion model)
// void MCLOdom::motion_update() {
//   position_mutex.lock();
//   double delta_x = tracker_odom->getPose().x - last_x;
//   double delta_y = tracker_odom->getPose().y - last_y;

//   // Apply Gaussian noise to particles for more realistic motion updates
//   for (MCLParticle particle : particles) {
//     particle.x += gaussian_random(delta_x, 0.1);
//     particle.y += gaussian_random(delta_y, 0.1);
//   }

//   // Update last odometry position
//   last_x = tracker_odom->getPose().x;
//   last_y = tracker_odom->getPose().y;
//   position_mutex.unlock();
// }

// // Resample particles based on their weights
// void MCLOdom::resample() {
//   position_mutex.lock();

//   // Find the maximum particle weight
//   double max_weight = 0;
//   for (MCLParticle particle : particles) {
//     if (particle.weight > max_weight) {
//       max_weight = particle.weight;
//     }
//   }

//   // particles to spread randomly
//   double UNIFORM_RANDOM_PARTICLES = uniform_random_percent * particle_count;

//   std::vector<MCLParticle> new_particles;

//   // Stochastic Universal Sampling
//   int index = floor(random() * particles.size());
//   double beta = 0; // accumulate random values scaled by max weight
//   for (int i = 0; i < particles.size() - UNIFORM_RANDOM_PARTICLES; i++) {
//     beta += random() * 2 * max_weight;
//     while (beta > particles[index].weight) {
//       beta -= particles[index].weight;
//       index = (index + 1) % particles.size();
//     }

//     new_particles.push_back(MCLParticle{
//         .x = particles[index].x, .y = particles[index].y, .weight = 1});
//   }

//   // resample remaning particles uniformly
//   for (int i = 0; i < UNIFORM_RANDOM_PARTICLES; i++) {
//     new_particles.push_back(MCLParticle{
//         .x = random() * FIELD_WIDTH, .y = random() * FIELD_WIDTH, .weight = 1});
//   }

//   // Update particle set
//   particles = new_particles;

//   position_mutex.unlock();
// }

// // Update particle weights based on sensor readings
// void MCLOdom::sensor_update() {
//   position_mutex.lock();

//   for (MCLParticle particle : particles) {
//     particle.weight = 1; // Initialize weight
//     for (MCLSensor sensor : sensors) {
//       // Adjust the sensor angle based on the particle's orientation
//       double theta = -tracker_odom->getPose(true).theta + sensor.theta_offset;

//       // Multiply particle's weight by likelihood from the sensor
//       particle.weight *=
//           get_particle_chance(particle.x, particle.y, sensor, theta);
//     }
//   }

//   position_mutex.unlock();
// }

// // Get estimated X coordinate of the robot
// double MCLOdom::get_x() {
//   position_mutex.lock();
//   double temp = predicted_x;
//   position_mutex.unlock();

//   return temp;
// }

// // Get estimated Y coordinate of the robot
// double MCLOdom::get_y() {
//   position_mutex.lock();
//   double temp = predicted_y;
//   position_mutex.unlock();

//   return temp;
// }

// // Get estimated position as a vector
// Vec MCLOdom::get_position() {
//   position_mutex.lock();
//   Vec temp = Vec(predicted_x, predicted_y);
//   position_mutex.unlock();

//   return temp;
// }

// // Get estimated pose (position and orientation)
// Pose MCLOdom::get_pose() {
//   position_mutex.lock();
//   double x = predicted_x;
//   double y = predicted_y;
//   position_mutex.unlock();
//   Angle rotation = get_rotation();

//   return Pose(x, y, rotation);
// }

// // Get the robot's current rotation
// Angle MCLOdom::get_rotation() {
//   return Angle::from_deg(tracker_odom->getPose().theta);
// }

// // Set the robot's rotation
// void MCLOdom::set_rotation(Angle rotation) {
//   tracker_odom->setPose(tracker_odom->getPose().x, tracker_odom->getPose().y, rotation.deg());
// }

// void MCLOdom::print_telemetry()
// {
//   std::cout << "[TELEMETRY.PF.DATA]:";
//   for (const auto &particle : particles)
//   {
//     std::cout << particle.x << "," << particle.y << "," << particle.weight << ";";
//   }
//   std::cout << std::endl;

//   Pose mcl_pose = get_pose();
//   std::cout << "[TELEMETRY.ODOM.MCL]:" << mcl_pose.x << "," << mcl_pose.y << "," << mcl_pose.deg() << std::endl;

//   lemlib::Pose original_pose = tracker_odom->getPose();
//   std::cout << "[TELEMETRY.ODOM.ORIGINAL]:" << original_pose.x << "," << original_pose.y << "," << original_pose.theta << std::endl;
// }

// // Start the MCL task for particle updates
// void MCLOdom::start_task(double initial_x, double initial_y,
//                                 Angle initial_theta) {
//   position_mutex.lock();
//   predicted_x = initial_x;
//   predicted_y = initial_y;

//   last_x = tracker_odom->getPose().x;
//   last_y = tracker_odom->getPose().y;
//   position_mutex.unlock();

//   tracker_odom->setPose(tracker_odom->getPose().x, tracker_odom->getPose().y, initial_theta.deg());

//   // Start particle update task
//   task = pros::Task([initial_x, initial_y, this] {
//     // Initialize particles
//     particles.push_back(
//         MCLParticle{.x = initial_x, .y = initial_y, .weight = 1});
//     for (int i = 1; i < particle_count; i++) {
//       particles.push_back(MCLParticle{.x = random() * FIELD_WIDTH,
//                                       .y = random() * FIELD_WIDTH,
//                                       .weight = 1});
//     }

//     while (true) {
//       motion_update(); // Update particle positions
//       resample();      // Resample particles
//       sensor_update(); // Update weights based on sensor data

//       position_mutex.lock();
//       predicted_x = 0;
//       predicted_y = 0;
//       double total_weight = 0;

//       // Compute weighted average of particles for estimated position
//       for (MCLParticle particle : particles) {
//         predicted_x += particle.x * particle.weight;
//         predicted_y += particle.y * particle.weight;
//         total_weight += particle.weight;
//       }

//       predicted_x /= total_weight;
//       predicted_y /= total_weight;
//       position_mutex.unlock();

//       print_telemetry(); // telemetry data

//       if (pros::Task::notify_take(true, 10) > 0) {
//         break; // Exit task if notified
//       }
//     }
//   });
// }

// // Stop the MCL task
// void MCLOdom::stop_task() {
//   task.notify();
//   task.join();
// }
