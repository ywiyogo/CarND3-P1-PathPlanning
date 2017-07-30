#include "cost_functions.h"
#include "vehicle.h"
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
    , v(20)
    , prev_v(0)
    , a(0)
    , prev_a(0)
    , behavior_state("KL") // KL
    ,                      // constant speed,
    max_acceleration(0)
{
  //  for(int i=0; i < 10; i++)
  //  {
  //    acc_list.push_back(0.0);
  //  }
}

// Vehicle::Vehicle(int lane, double s, double v, double a)
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



/*
 * 0<d<4 is lane 0
   4<d<8 is lane 1
   8<d<12 is lane 2
 */
int Vehicle::get_lane(double d)
{
  if(d < 4) {
    return 0;
  } else if(d < 8) {
    return 1;
  } else {
    return 2;
  }
}

void Vehicle::do_prediction(vector<double> &result, string behavior_state, double dt_s=1)
{
  // s, d, v, a, yaw, cost
  //vector<double> prediction_state(6, 0.0);
  if (behavior_state.compare("KL"))
  {
    result[0] = this->s + this->v * dt_s;
    result[1] = this->d;
    if(this->v< 13){
      result[2] = this->v + 1;
    }else{
      result[2] = 12;
    }
    result[3] = 1;
    result[4] = this->yaw + 5.;
  }else if (behavior_state.compare("LCL"))
  {
    result[0] = this->s + this->v * dt_s;
    result[1] = this->d - 4;
    result[2] = this->v;
    result[3] = this->a - 1;
    result[4] = this->yaw + 5.;
  }else if (behavior_state.compare("LCR"))
  {
    result[0] = this->s + this->v * dt_s;
    result[1] = this->d + 4;
    result[2] = this->v;
    result[3] = this->a - 1;
    result[4] = this->yaw - 10;
  }
  else{//slow down the vehicle
    result[2] = this->v - 2; //- 2m/s
    result[0] = this->s + result[2] * dt_s;
    result[1] = this->d;
    result[3] = this->a - 1;
    result[4] = this->yaw;
  }
}

void Vehicle::realize_state(const vector<double> &predicted_state)
{
  double dist_inc = predicted_state[2] / 50.;
  cout<<"Distance inc: "<<dist_inc<<" yaw: "<<predicted_state[4]<<endl;
  for(int i = 0; i < 50; i++) {
    this->next_x_vals.push_back(this->x + (dist_inc * i) * cos(deg2rad(predicted_state[4])));
    this->next_y_vals.push_back(this->y + (dist_inc * i) * sin(deg2rad(predicted_state[4])));
  }
}

// void Vehicle::update_state(map<int, vector<vector<int> > > predictions)
void Vehicle::update_state(double x,
    double y,
    double s,
    double d,
    double v,
    double yaw,
    const vector<vector<int>> &pred_vehicles)
{
  // updated car states
  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->prev_v = this->v;
  this->v = v;
  this->yaw = yaw;
  this->a = (v - this->prev_v) / 0.2;

  // behavior check
  vector<string> statelist = { "KL", "LCL", "LCR", "PLCL", "PLCR" };
  this->lane = get_lane(d);
  if(this->lane == 2) {
    statelist.erase(statelist.begin() + 2);
    statelist.erase(statelist.begin() + 4);
  } else if(this->lane == 0) {
    statelist.erase(statelist.begin() + 1);
    statelist.erase(statelist.begin() + 3);
  }
  
  // do a prediction and calculate cost for all states
  vector<vector<double> > possible_states;
  
  for(int i = 0; i < statelist.size(); i++) {
    vector<double> predicted_state(6, 0.0);
    do_prediction(predicted_state, statelist[i]);
    // assign the cost
    double cost = CostFunctions::calculate_cost(predicted_state, pred_vehicles);
    predicted_state[5] = cost;
    // append to the vector

    possible_states.push_back(predicted_state);

  }
  
  // find the minimal cost
  double mincost = 9999;
  int min_i;
  for(int i = 0; i < possible_states.size(); i++) {

    if(possible_states[i][5] < mincost) {
      mincost = possible_states[i][5];
      min_i = i;
      
    }
  }
  cout << "Min State: " << statelist[min_i] << " has minimal cost: " << mincost << endl;
  
  // realize the prediction state
  realize_state(possible_states[min_i]);
  
  
  this->acc_list.push_back(this->a);
  if(acc_list.size() >= 10) {
    for(int i = 0; i < acc_list.size(); i++) {
      this->jerk += acc_list[i];
    }
    this->jerk = this->jerk / acc_list.size();
    this->acc_list.pop_front();
  } else {
    // use the old jerk value
  }

  cout << "x: " << this->x << " y: " << this->y << " s: " << this->s << " d: " << this->d << " yaw: " << this->yaw
       << " speed: " << this->v << endl;
  cout << "a: " << this->a << " jerk: " << this->jerk << endl;
  cout <<"--------------------------------------------"<<endl;
}