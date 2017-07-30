#ifndef VEHICLE_H
#define VEHICLE_H
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <deque> 

using namespace std;

class Vehicle
{
  public:
  struct collider {

    bool collision; // is there a collision?
    int time;       // time collision happens
  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;
  double x,y;
  double s,d,yaw;
  double v;
  double prev_v;
  double a;
  double prev_a;
  double target_speed;
  int lanes_available;
  double max_acceleration;
  double jerk;

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  deque<double> acc_list;
  
  string behavior_state;
  
  /**
  * Constructor
  */
//  Vehicle(int lane, double s, double v, double a);
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();
  int get_lane(double d);
  void update_state(double x, double y, double s, double d, double v, double yaw, const vector<vector<int>> &pred_vehicles);
  void realize_state(const vector<double> &predicted_state);
  void do_prediction(vector<double> &result, string behavior_state, double dt_s);
  vector<int> state_at(int t);
  
  double deg2rad(double x) { return x * M_PI / 180; }
  double rad2deg(double x) { return x * 180 / M_PI; }
  
};

#endif