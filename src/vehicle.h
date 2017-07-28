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
  double s,d, yaw;
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
  vector<double> acc_list;

  string state;

  /**
  * Constructor
  */
//  Vehicle(int lane, double s, double v, double a);
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_state(vector<vector<int> > predictions);

  void configure(vector<int> road_data);

  string display();

  void increment(int dt);

  vector<int> state_at(int t);
  
  double deg2rad(double x) { return x * M_PI / 180; }
  double rad2deg(double x) { return x * 180 / M_PI; }
  
  bool collides_with(Vehicle other, int at_time);

  collider will_collide_with(Vehicle other, int timesteps);

  void realize_state(map<int, vector<vector<int> > > predictions);

  void realize_constant_speed();

  int _max_accel_for_lane(map<int, vector<vector<int> > > predictions, int lane, int s);

  void realize_keep_lane(map<int, vector<vector<int> > > predictions);

  void realize_lane_change(map<int, vector<vector<int> > > predictions, string direction);

  void realize_prep_lane_change(map<int, vector<vector<int> > > predictions, string direction);

  vector<vector<int> > generate_predictions(int horizon);
};

#endif