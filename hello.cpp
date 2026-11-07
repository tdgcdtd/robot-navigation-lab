#include <iostream>

int main(){
    double robot_x = 2.5;
    double step = 0.75;
    robot_x = robot_x + step;
    std::cout << "robot_x = " << robot_x << '\n';
    return 0;
}
