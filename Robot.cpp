#include "Robot.h"
#include <iostream>
#include <cmath>

constexpr double PI = 3.1415926;

Robot::Robot(double initial_x, double initial_y, double initial_speed, double initial_heading)
    :x(initial_x),
     y(initial_y),
     speed(0),
     heading(0)
{
    setSpeed(initial_speed);
    setHeading(initial_heading);
}
bool Robot::setSpeed(double new_speed)
{
    if(new_speed >=0 && new_speed <= 5)
    {
        speed = new_speed;
        return true;
    }
    else
    {
        return false;
    }
}
void Robot::setHeading(double new_heading)
{
     heading = std::fmod(new_heading, 2 * PI);

     if(heading < 0)
     {
        heading += 2 * PI;
     }
}
void Robot::turn(double delta_heading)
{
    setHeading(heading + delta_heading);
}
bool Robot::move(double time)
{
    if(time >= 0)
    {
        double distance = speed * time;

        x += std::cos(heading) * distance;
        y += std::sin(heading) * distance;
        return true;
    }
    else
    {
        return false;
    }
}
void Robot::stop()
{
    speed = 0;
}
RobotState Robot::getState() const
{
    return {x, y, speed, heading};
}

void printRobotState(const RobotState& state)
{
    std::cout << "x: " << state.x << 
                 ", y: " << state.y << 
                 ", speed: " << state.speed <<
                 ", heading: " << state.heading << '\n';
}