#pragma once

struct RobotState
{
    double x;
    double y;
    double speed;
    double heading;
};

class Robot
{
private:
    double x;
    double y;
    double speed;
    double heading;

public:
    Robot(double initial_x, double initial_y, double initial_speed, double initial_heading);

    bool setSpeed(double new_speed);
    void setHeading(double new_heading);
    void turn(double delta_heading);
    bool move(double time);
    void stop();
    RobotState getState() const;
};

void printRobotState(const RobotState& state);