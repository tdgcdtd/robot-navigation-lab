#include <iostream>
#include <vector>
#include "speed_utils.h"

int main()
{
    std::vector<double> speeds;

    speeds.push_back(1.2);
    speeds.push_back(2.5);
    speeds.push_back(3.8);

    std::cout << "Number of speeds: " << speeds.size() << "\n";
    std::cout << "First speed: " << speeds[0] << "\n";
    std::cout << "Last speed: " << speeds[2] << "\n";

    speeds[1] = 3.0;

    for(double speed: speeds)
    {
        std::cout << speed << '\n';
    }

    doubleSpeeds(speeds);
    for(double speed: speeds)
    {
        std::cout << speed << '\n';
    }

    return 0;
}