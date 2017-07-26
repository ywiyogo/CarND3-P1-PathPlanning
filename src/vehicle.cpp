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
Vehicle::Vehicle(int lane, double s, double v, double a)
{

  this->lane = lane;
  this->s = s;
  this->v = v;
  this->prev_v = 0;
  this->a = a;
  this->prev_a = 0;
  state = "CS";
  max_acceleration = -1;
  this->acc_list(10);
}

Vehicle::~Vehicle()
{
}

void Vehicle::update_state(map<int, vector<vector<int> > > predictions)
{

  // speed
  double dist_inc = 0.1;
  for(int i = 0; i < 50; i++) {
    next_x_vals.push_back(car_x + (dist_inc * i) * cos(deg2rad(car_yaw)));
    next_y_vals.push_back(car_y + (dist_inc * i) * sin(deg2rad(car_yaw)));
  }
  this->a = this->prev_v - this->v ;

  this->acc_list.erase(acc_list.begin());
  this->acc_list.push_back(this->a);
  for(int i = 0; i < acc_list.size(); i++) {
    this->jerk += acc_list[i];
  }
  this->jerk = this->jerk / acc_list.size();
  cout << "Jerk: " << this->jerk << endl;
}