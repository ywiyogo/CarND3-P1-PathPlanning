#ifndef BEHAVIORFSM_HPP_INCLUDED
#define BEHAVIORFSM_HPP_INCLUDED

#include "prediction.h"
#include "sdvehicle.h"
#include <deque>
#include <string>
#include <vector>
#include <chrono>

using namespace std;

struct MinCost{
  int lane;
  double cost;
};

enum Lane{
  LANE0 = 0,
  LANE1 = 1,
  LANE2 = 2,
};
enum DriveMode
{
  STOP = 0,
  NORMAL = 1,
  CAUTIOUS = 2,
  ALERT = 3,
};

// ----------------------------------------------------------------------------
// BehaviorFSM (FSM base class) declaration
//
class SDVehicle;
//-----------------------------------
// Abstract Class
//-----------------------------------
class BehaviorFSM
{
  public:
  BehaviorFSM();
  BehaviorFSM(const string& name, int lane);
  virtual ~BehaviorFSM();


  /*
  * Current environment measurements update
  */
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory) = 0;

  protected:

  /*
  *
  */
  void set_behavior_state(SDVehicle& sdcar, BehaviorFSM* state);

  /*
  * Generate a trajectory for the next prediction steps
  */
  void generate_trajectory(SDVehicle& sdcar, double acc, double goal_d, vector<double> &s_coeffs, vector<double> &d_coeffs, double T = 2);

  /*
  * Perform the motion trajectory from JMT
  */
  void realize_behavior(SDVehicle& sdcar, const vector<double> &s_coeff, const vector<double> &d_coeff, int steps = 40);


  /*
  * Find the closest front and behind vehicles from a given list of vehicles e in a lane
  */
  void find_closest_cars_inlane(double ego_s,
      const vector<deque<Vehicle> >& inlane_veh_trajectories,
      deque<Vehicle>& res_frontcar,
      deque<Vehicle>& res_rearcar);

  /*
  * Calculate the cost of the given set of vehicle of a lane (can be different lane)
  */
  double calc_behaviorlane_cost(SDVehicle& sdcar, int lane, vector<deque<Vehicle> >& inlane_veh_trajectories);

  MinCost calc_min_cost(SDVehicle& sdcar,map<int, deque<Vehicle> > predictions, int curr_lane);

  // Name of the FSM
  string name_;
  // Goal lane of current state
  int goallane_;
  // drive mode 0: normal, 1: cautious 2: alert
  unsigned drive_mode_;
};

//-----------------------------------
// Behavior States
//-----------------------------------
class Ready : public BehaviorFSM
{
  public:
  Ready(const string& name, int lane)
      : BehaviorFSM(name, lane){};
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~Ready();
};

class KeepLane : public BehaviorFSM
{
  public:
  KeepLane(const string& name, int lane)
      : BehaviorFSM(name, lane){};

  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~KeepLane();
};

class LCL : public BehaviorFSM
{
  public:
  LCL(const string& name, int lane)
      : BehaviorFSM(name, lane){};
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~LCL();
};

class PrepareLCL : public BehaviorFSM
{
  public:
  PrepareLCL(const string& name, int lane)
      : BehaviorFSM(name, lane){};
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~PrepareLCL();
};

class LCR : public BehaviorFSM
{
  public:
  LCR(const string& name, int lane)
      : BehaviorFSM(name, lane){};
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~LCR();
};

class PrepareLCR : public BehaviorFSM
{
  public:
  PrepareLCR(const string& name, int lane)
      : BehaviorFSM(name, lane){};
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~PrepareLCR();
};


#endif
