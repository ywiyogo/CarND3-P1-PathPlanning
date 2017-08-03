#ifndef BEHAVIORFSM_HPP_INCLUDED
#define BEHAVIORFSM_HPP_INCLUDED

#include "prediction.h"
#include "sdvehicle.h"
#include <deque>
#include <string>
#include <vector>
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
  virtual ~BehaviorFSM();
  
  virtual void update_ego(SDVehicle& sdcar, EgoVehicle& ego, const vector<double> &prev_path_x, const vector<double> &prev_path_y);
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory) = 0;
  

protected:
  int counter;
  string name;
  virtual void entry(SDVehicle& sdcar);
  int get_lane(double d);
  void set_behavior_state(SDVehicle& sdcar, BehaviorFSM* state);
  void realize_behavior(SDVehicle& sdcar, vector<double> s_coeff);
  void find_closest_cars_inlane(double ego_s,
      const map<int, deque<Vehicle> >& inlane_veh_trajectories,
      vector<deque<Vehicle> >& result);
  double calc_behaviorlane_cost(SDVehicle& sdcar, vector<deque<Vehicle> >& inlane_veh_trajectories);
};

//-----------------------------------
// Behavior States
//-----------------------------------
class Ready : public BehaviorFSM
{
  public:
  Ready();
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~Ready();
};

class KeepLane : public BehaviorFSM
{
  public:
  KeepLane();
  virtual void entry(SDVehicle& sdcar);
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~KeepLane();
};

class LCL : public BehaviorFSM
{
  public:
  LCL();
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~LCL();
};

class PrepareLCL : public BehaviorFSM
{
  public:
  PrepareLCL();
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~PrepareLCL();
};

class LCR : public BehaviorFSM
{
  public:
  LCR();
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory);
  virtual ~LCR();
};

class PrepareLCR : public BehaviorFSM
{
  public:
  PrepareLCR();
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
