#ifndef BEHAVIORFSM_HPP_INCLUDED
#define BEHAVIORFSM_HPP_INCLUDED

#include "prediction.h"
#include "sdvehicle.h"
#include <deque>
#include <string>
#include <vector>
#include <chrono>

using namespace std;

struct EgoVehicle;

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
  * Self driving car state update
  */
  virtual void update_ego(SDVehicle& sdcar, Vehicle& ego, const vector<double>& prev_path_x, const vector<double>& prev_path_y);
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
  void generate_trajectory(SDVehicle& sdcar, double acc, double goal_d, vector<double> &s_coeffs, vector<double> &d_coeffs, double dt = 1);

  /*
  * Perform the motion trajectory from JMT
  */
  void realize_behavior(SDVehicle& sdcar, const vector<double> &s_coeff, const vector<double> &d_coeff, double dt);
  
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
  double calc_behaviorlane_cost(SDVehicle& sdcar, vector<deque<Vehicle> >& inlane_veh_trajectories);

  // Name of the FSM
  string name_;
  // Goal lane of current state
  int goallane_;
  // Suggested acceleration of current state
  double suggest_acc_;
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

// class BehaviorFSM
//{
//  /* NOTE: react(), entry() and exit() functions need to be accessible
//   * from tinyfsm::Fsm class. You might as well declare friendship to
//   * tinyfsm::Fsm, and make these functions private:
//   *
//   * friend class Fsm;
//   */
// public:
//
//  /* default reaction for unhandled events */
//  void react(tinyfsm::Event const &) { };
//  virtual void react(MinCost const &);
//
//  virtual void entry(void) { };  /* entry actions in some states */
//  void         exit(void)  { };  /* no exit actions at all */
//
//  void get_coeffs(vector<double> &coeffs);
//
//  Trajectory traj;
//  vector<double> s_coeff;
//  vector<double> d_coeff;
// protected:
//  vector<double> start_s;
//  vector<double> start_d;
//
//  vector<deque<Vehicle>> veh_pred_trajectories;
//};

#endif
