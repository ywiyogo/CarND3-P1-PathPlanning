#include "vehicle.h"
#include <iostream>
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <string>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle()
    : lane(0)
    , x(0)
    , y(0)
    , s(0)
    , d(0)
    , yaw(0)
    , v(0)
    , prev_v(0)
    , a(0)
    , prev_a(0)
    , state("KL")
    , // constant speed,
    max_acceleration(0)
    , acc_list(10)
{
}

//Vehicle::Vehicle(int lane, double s, double v, double a)
//    : lane(lane)
//    , x(0)
//    , y(0)
//    , s(0)
//    , d(0)
//    , yaw(0)
//    , s(s)
//    , v(v)
//    , prev_v(0)
//    , a(a)
//    , prev_a(0)
//    , state("CS")
//    , max_acceleration(0)
//    , acc_list(10, 0.0)
//{
//  
//}

Vehicle::~Vehicle()
{
}

//void Vehicle::update_state(map<int, vector<vector<int> > > predictions)
void Vehicle::update_state(vector<vector<int>> predictions)
{

  // speed
  double dist_inc = 0.1;
  for(int i = 0; i < 50; i++) {
    next_x_vals.push_back(this->x + (dist_inc * i) * cos(deg2rad(this->yaw)));
    next_y_vals.push_back(this->y + (dist_inc * i) * sin(deg2rad(this->yaw)));
  }
  this->a = this->prev_v - this->v;

  this->acc_list.erase(acc_list.begin());
  this->acc_list.push_back(this->a);
  for(int i = 0; i < acc_list.size(); i++) {
    this->jerk += acc_list[i];
  }
  this->jerk = this->jerk / acc_list.size();
  cout << "Jerk: " << this->jerk << endl;
}