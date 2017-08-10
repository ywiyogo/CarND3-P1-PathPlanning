#ifndef SDVEHICLE_H
#define SDVEHICLE_H
#include "behaviorfsm.h"
#include "prediction.h"
#include <deque>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <random>
#include <sstream>
#include <string>
#include <vector>
#include <chrono>
using namespace std;


struct EgoVehicle{
  double x;
  double y;
  double s;
  double d;
  double v_ms;
  double yaw;
};

// Implementing State Design Pattern
class BehaviorFSM;

//inherited from Vehicle class due to several same attributes
class SDVehicle:public Vehicle
{
  friend class BehaviorFSM;

  public:

  double s_dot, d_dot;
  double s_dotdot, d_dotdot;
  double prev_v;
  double jerk;
  
  int prev_path_size;
  vector<double> next_x_vals;
  vector<double> next_y_vals;
  // car trajectory in global CS
  vector<double> global_traj_x_;
  vector<double> global_traj_y_;
  deque<double> acc_list;
  
  double sim_delay;
  double ref_v_;
  /**
  * Constructor
  */
  SDVehicle();
  SDVehicle(int lane, double s, double v);
  /**
  * Destructor
  */
  virtual ~SDVehicle();

  // update the self driving car state
  void update_ego(Vehicle &ego, const vector<double> &prev_path_x, const vector<double> &prev_path_y, double dt);
  
  //update measurements of the environment
  void update_env(const map<int, deque<Vehicle> >& vehicle_trajectories, double dt);
  
  //Drive the car until reach the given velocity and d
  void drive(double goal_v, double goal_d);

  // Copy the map waypoints
  void set_map_waypoints_x(const vector<double> &mwaypoints_x);
  void set_map_waypoints_y(const vector<double> &mwaypoints_y);
  void set_map_waypoints_s(const vector<double> &mwaypoints_s);
  void set_map_waypoints_dx(const vector<double> &mwaypoints_dx);
  void set_map_waypoints_dy(const vector<double> &mwaypoints_dy);
  
  vector<double> jerk_min_trajectory(vector<double> start, vector<double> end, double T);

/*
* Transform Frenet point to the (x,y) point in Cartesian coordinate
*/
  vector<double> getXY(double s, double d);


  private:
  BehaviorFSM* behaviorfsm_;
  vector<double> map_wp_x;
  vector<double> map_wp_y;
  vector<double> map_wp_s;
  vector<double> map_wp_dx;
  vector<double> map_wp_dy;
};

#endif