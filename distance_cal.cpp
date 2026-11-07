#include <iostream>

double calculate_distance(double speed, double time)
{
   return speed * time;
}

bool isSpeedok(double speed)
{
   return speed >= 0 && speed <= 5;
}

bool isTimeok(double time)
{
   return time >= 0 && time <= 60;
}

int main(){
    double speed = 0;
    double time = 0;
    double distance = 0;
    
   while(true)
   {
      std::cout << "please enter robot speed and running time : ";
      std::cin >> speed >> time;

      bool speed_ok = isSpeedok(speed);
      bool time_ok = isTimeok(time);

      if(speed_ok && time_ok)
      {
         break;
      }      
      if(!speed_ok){
         std::cout << "Error: speed must be between 0 and 5 m/s.\n";
      }
      if(!time_ok)
      {
         std::cout << "Error: time must be between 0 and 60 s.\n";
      }
   }

   distance = calculate_distance(speed, time);

   std::cout << "Robot travelled " << distance << " m\n";

    return 0;
}
