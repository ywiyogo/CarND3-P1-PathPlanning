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
using namespace std;


struct EgoVehicle{
  double x;
  double y;
  double s;
  double d;
  double v_ms;
  double yaw;
};

// Implementing State Pattern
class BehaviorFSM;

class SDVehicle:public Vehicle
{
  friend class BehaviorFSM;

  public:

  double s_dot, d_dot;
  double s_dotdot, d_dotdot;
  double prev_v;
  double jerk;

  vector<double> next_x_vals;
  vector<double> next_y_vals;
  vector<double> prev_path_x;
  vector<double> prev_path_y;
  deque<double> acc_list;

  /**
  * Constructor
  */

  SDVehicle();
  SDVehicle(int lane, double s, double v);
  /**
  * Destructor
  */
  virtual ~SDVehicle();

  void update_ego(EgoVehicle &egovehicle, const vector<double> &prev_path_x, const vector<double> &prev_path_y);
  void update_env(const map<int, deque<Vehicle> >& vehicle_trajectories);
  void realize_state(const vector<double>& predicted_state);
  void set_map_waypoints_x(const vector<double> &mwaypoints_x);
  void set_map_waypoints_y(const vector<double> &mwaypoints_y);
  void set_map_waypoints_s(const vector<double> &mwaypoints_s);
  vector<double> jerk_min_trajectory(vector<double> start, vector<double> end, double T);
  void calc_best_trajectory(vector<double> start_s, vector<double> start_d, int target_veh_id, vector<double> delta, int T, vector<vector<double>> predictions);
  vector<double> getXY(double s, double d);
  vector<double> getXY(double s, double d, vector<double> maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
  //  void do_prediction(vector<double> &result, string behavior_state, double dt_s);
  vector<int> state_at(int t);

  private:
  BehaviorFSM* behaviorfsm_;
  vector<double> map_wp_x;
  vector<double> map_wp_y;
  vector<double> map_wp_s;
};

#endif