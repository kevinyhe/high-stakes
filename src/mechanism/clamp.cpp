#include "mechanism/clamp.hpp"
#include "main.h"
#include "config.hpp"

namespace mechanism
{
    std::unique_ptr<Clamp> Clamp::instance = nullptr;
    std::once_flag Clamp::init_flag;

    Clamp::Clamp(std::shared_ptr<Pneumatic> pneumatic, std::shared_ptr<pros::Distance> distance, double autoclamp_threshold) 
        : m_pneumatic(pneumatic), m_distance(distance), m_autoclamp_threshold(autoclamp_threshold)
    {
        start_task();
    }

    Clamp::~Clamp()
    {
        stop_task();
    }

    void Clamp::start_task()
    {
        mutex.lock();
        if (task_on_flag)
        {
            mutex.unlock();
            return;
        }

        task_on_flag = true;
        mutex.unlock();

        // Clamp_rotation_sensor.reset();

        task = pros::Task([this]()
                          {
            while (true)
            {
                mutex.lock();
                if (task_on_flag == false)
                {
                    break;
                }
                                
                if (m_autoclamp)
                {
                    if (m_distance->get() < m_autoclamp_threshold)
                    {
                        if (m_autoclamp_start_time == 0) {
                            m_autoclamp_start_time = pros::millis();
                        }
                        if (pros::millis() - m_autoclamp_start_time > 50 && m_autoclamp_start_time != 0)
                        {
                            m_pneumatic->extend();
                            m_autoclamp_start_time = 0;
                        }
                    }
                    else {
                        m_autoclamp_start_time = 0;
                    }
                }
                mutex.unlock();

                pros::delay(20);
            } });
    }

    void mechanism::Clamp::stop_task()
    {
        mutex.lock();
        task_on_flag = false;
        mutex.unlock();
    }

    void Clamp::extend()
    {
        m_pneumatic->extend();
    }

    void Clamp::retract()
    {
        m_pneumatic->retract();
    }

    void Clamp::toggle()
    {
        if (m_pneumatic->get_value())
        {
            retract();
        }
        else
        {
            extend();
        }
    }

    bool Clamp::get_autoclamp() {
        mutex.lock();
        bool temp = this->m_autoclamp;
        mutex.unlock();
        return temp;
    }

    bool Clamp::get_value() {
        mutex.lock();
        bool temp = this->m_pneumatic->get_value();
        mutex.unlock();
        return temp;
    }

    void Clamp::set_autoclamp(bool autoclamp) {
        mutex.lock();
        this->m_autoclamp = autoclamp;
        mutex.unlock();
    }
} // namespace mechanism