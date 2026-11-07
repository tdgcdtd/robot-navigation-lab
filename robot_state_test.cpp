#include "Robot.h"
#include <iostream>
#include <vector>

constexpr double PI = 3.1415926;

enum class Command
{
    Move,
    Turn,
    Stop
};

void executeCommand(Robot& robot, Command command)
{
    switch(command)
    {
        case Command::Move:
            robot.move(1.0);
            break;
        
        case Command::Turn:
            robot.turn(PI / 2);
            break;

        case Command::Stop:
            robot.stop();
            break;
    }
}

int main()
{
    Robot robot(0, 0, 2, 0);
    std::vector<RobotState> history;
    
    history.push_back(robot.getState());
    robot.move(2);
    history.push_back(robot.getState());
    robot.turn(PI/2);
    robot.move(3);
    history.push_back(robot.getState());
    robot.setSpeed(1);
    robot.turn(PI/2);
    robot.move(4);
    history.push_back(robot.getState());
    robot.stop();
    history.push_back(robot.getState());

    for(const RobotState& state: history)
    {
        printRobotState(state);
    }

    return 0;
}