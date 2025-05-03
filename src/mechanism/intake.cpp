#include "intake.hpp"
#include "config.hpp"

namespace mechanism
{
    std::unique_ptr<Intake> Intake::instance = nullptr;
    std::once_flag Intake::init_flag;

    Intake::Intake(std::shared_ptr<pros::Motor> motor, std::shared_ptr<pros::Optical> optical_sensor, std::shared_ptr<pros::Distance> distance_sensor, std::int32_t sort_distance, double red_bound, double blue_bound)
        : m_motor(motor), m_optical_sensor(optical_sensor), m_distance_sensor(distance_sensor), m_sort_distance(sort_distance), m_red_bound(red_bound), m_blue_bound(blue_bound), m_sort_enabled(false), m_sort_colour(RingColours::NONE), m_colour_state_detector(RingColours::NONE), m_ring_state_detector()
    {
        start_task();
    }

    Intake::~Intake()
    {
        stop_task();
    }

    void Intake::enable_sort(RingColours colour)
    {
        m_optical_sensor->set_integration_time(20); // Decrease Update time
       
        m_sort_enabled = true;  // Enable Sort
        m_sort_colour = colour; // Set Sort Color
    }
    void Intake::disable_sort()
    {
        m_optical_sensor->set_led_pwm(0); // Turn off light

        m_sort_enabled = false; // Disable sort
    }

    Intake::RingColours Intake::get_sort_colour() { return m_sort_colour; }
    Intake::RingColours Intake::get_current_ring_colour()
    {
        if (m_possession.size() > 0)
        {
            return m_possession[0];
        }
        else
        {
            return RingColours::NONE;
        }
    }

    double Intake::get_possession_count() { return m_possession.size(); }
    std::vector<Intake::RingColours> Intake::get_possession() { return m_possession; }

    void Intake::start_task()
    {
        mutex.lock();
        if (task_on_flag)
        {
            mutex.unlock();
            return;
        }

        task_on_flag = true;
        mutex.unlock();

        task = pros::Task([this]()
                          {        
            while (true)
            {
                mutex.lock();
                if (task_on_flag == false)
                {
                    break;
                }

                // get arm instance 
                auto& arm = Arm::get_instance();

                if (m_sort_enabled)
                {
                    m_optical_sensor->set_led_pwm(100);         // Turn on LED
                    double color = m_optical_sensor->get_hue(); // Grab Color from sensor
                    int proximity = m_optical_sensor->get_proximity();
                    RingColours ring_colour;

                    // std::cout << "Proximity: " << proximity << std::endl;

                    // Calculate Current Ring Color
                    if (color < m_red_bound && proximity > 180)
                    {
                        ring_colour = RingColours::RED;
                    }
                    else if (color > m_blue_bound && proximity > 180)
                    {
                        ring_colour = RingColours::BLUE;
                    }
                    else
                    {
                        ring_colour = RingColours::NONE;
                    }

                    // Check if color changed
                    m_colour_state_detector.check(ring_colour);
                    if (m_colour_state_detector.getChanged())
                    {
                        if (m_colour_state_detector.getValue() != RingColours::NONE) // if colour is sorted colour
                        {
                            m_possession.push_back(m_colour_state_detector.getValue()); // Add new ring to possession
                        }
                        if (m_stop_next_ring_flag && this->get_current_ring_colour() != m_sort_colour)
                        {
                            m_stop_next_ring_flag = false;
                            // stop intake
                            this->state = IntakeState::DISABLED;
                        }
                    }
                    // print ring colour to pros lcd
                    pros::lcd::print(6, "Ring Colour: %d", static_cast<int>(m_colour_state_detector.getValue()));
                    pros::lcd::print(4, "Possession: %d", m_possession.size()); // debug info

                    std::int32_t distance = m_distance_sensor->get_distance(); // Grab Distance from distance sensor

                    // Update Change Detector with new value
                    m_ring_state_detector.check(distance < m_sort_distance);
                    // If ring state has changed to detected
                    if (m_ring_state_detector.getChanged() && m_ring_state_detector.getValue())
                    {

                        // If sort is active and color is wrong, remove ring
                        if (m_sort_enabled && this->get_current_ring_colour() == m_sort_colour)
                        {
                            // m_pSort->extend();
                            // this->sort_start_time= pros::millis();
                            // // we have to check if the prev state was a dejam else it gets stuck in a cycle
                            // pros::lcd::print(5, "SORTING: %d", static_cast<int>(this->get_current_ring_colour()));
                            // this->pre_sort_state = (this->state == IntakeState::SORT || this->state == IntakeState::DEJAM) ? IntakeState::DISABLED : this->state;
                            // this->state = IntakeState::SORT;
                            pros::delay(70);
                            this->dejam_start_time = pros::millis();
                            // we have to check if the prev state was a dejam else it gets stuck in a cycle
                            this->pre_dejam_state = this->state == IntakeState::DEJAM ? IntakeState::DISABLED : this->state;
                            this->state = IntakeState::DEJAM;
                        }
                        if (m_possession.size() > 0)
                        {
                            m_possession.erase(m_possession.begin()); // Remove ring from count
                        }
                    }
                    // If ring state has changed to nothing detected
                    else if (m_ring_state_detector.getChanged() && !m_ring_state_detector.getValue())
                    {
                        // do nothing
                    }
                }
                else if (!m_sort_enabled)
                {
                    m_optical_sensor->set_led_pwm(0); // Turn off light
                }

                // Check if the motors are at 0 velocity
                if (std::abs(m_motor->get_actual_velocity()) < 20 && m_motor->get_current_draw() > 2000 &&  this->state != IntakeState::DISABLED && this->state != IntakeState::DEJAM && arm.get_state() != ArmState::LOAD)
                {
                    // If the motors are at 0 velocity, start or update the timer
                    if (zero_velocity_start_time == 0)
                    {
                        zero_velocity_start_time = pros::millis();
                    }
                    else if (pros::millis() - zero_velocity_start_time > 1000)
                    {
                        // If the motors have been at 0 velocity for more than 500 ms, activate dejam state
                        this->dejam_start_time = pros::millis();
                        this->pre_dejam_state = this->state == IntakeState::DEJAM ? IntakeState::DISABLED : this->state;
                        this->state = IntakeState::DEJAM;
                        zero_velocity_start_time = 0; // reset timer
                    }
                }
                else
                {
                    // If the motors are not at 0 velocity, reset the timer
                    zero_velocity_start_time = 0;
                }

                // If jammed for a certain amount of time return to initial state
                if (this->state == IntakeState::DEJAM && pros::millis() - this->dejam_start_time > 200)
                {
                    this->state = this->pre_dejam_state; // return to initial state
                    this->dejam_start_time = 0; // reset
                }
                if (this->state == IntakeState::SORT && pros::millis() - this->sort_start_time > 200)
                {
                    this->state = this->pre_sort_state; // return to initial state
                    this->sort_start_time = 0; // reset
                }
                
                // if the arm is in the wall stake position, return to initial state after a certain amount of time
                if (this->state == IntakeState::WALL_STAKE)
                {
                    if (pros::millis() - wall_stake_start_time > 100 && wall_stake_start_time != 0)
                    {
                        this->state = IntakeState::DISABLED;
                        wall_stake_start_time = 0; // reset
                    }
                    else if (wall_stake_start_time == 0) {
                        wall_stake_start_time = pros::millis(); // update timer
                    }
                }
                else {
                    wall_stake_start_time = 0;
                }

                IntakeState current_state = this->state;
                mutex.unlock();
                
                switch (current_state) {
                    case IntakeState::HOOK:
                        m_motor->move(127);
                        // pros::lcd::print(6, "HOOK");
                        break;
                    case IntakeState::WALL_STAKE:
                        m_motor->move(-40);
                        // pros::lcd::print(6, "WALL_STAKE");
                        break;
                    case IntakeState::REVERSE:
                        m_motor->move(-127);
                        // pros::lcd::print(6, "REVERSE");
                        break;
                    case IntakeState::DEJAM:
                        m_motor->move(-127);
                        // pros::lcd::print(6, "DEJAM");
                        break;
                    case IntakeState::SORT:
                        m_motor->move(-127);
                        // pros::lcd::print(6, "DEJAM");
                        break;
                    case IntakeState::DISABLED:
                        m_motor->move(0);
                        // pros::lcd::print(6, "DISABLED");
                        break;
                }

                pros::delay(20);
            } });
    }

    void Intake::enable_stop_next_ring() {
        mutex.lock();
        m_stop_next_ring_flag = true;
        mutex.unlock();
    }

    void Intake::disable_stop_next_ring() {
        mutex.lock();
        m_stop_next_ring_flag = false;
        mutex.unlock();
    }

    void Intake::stop_task()
    {
        mutex.lock();
        task_on_flag = false;
        mutex.unlock();
    }

    void Intake::set_state(IntakeState new_state)
    {
        mutex.lock();
        if (new_state == IntakeState::DISABLED)
        {
            m_motor->move(0);
        }
        state = new_state;
        mutex.unlock();
    }

    IntakeState Intake::get_state()
    {
        mutex.lock();
        auto temp = state;
        mutex.unlock();
        return temp;
    }
} // namespace mechanism