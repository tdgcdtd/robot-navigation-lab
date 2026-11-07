#include "speed_utils.h"

void doubleSpeeds(std::vector<double>& speeds)
{
    for(double& speed: speeds)
    {
        speed = speed * 2;
    }
}