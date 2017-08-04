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
  BehaviorFSM(const string& name, int lane);
  virtual ~BehaviorFSM();

  virtual void
  update_ego(SDVehicle& sdcar, EgoVehicle& ego, const vector<double>& prev_path_x, const vector<double>& prev_path_y);
  virtual void update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > trajectory) = 0;

  protected:
  string name_;
  int goallane_;
  void set_behavior_state(SDVehicle& sdcar, BehaviorFSM* state);
  void generate_trajectory(SDVehicle& sdcar,  double goal_s_dot, double goal_s_dotdot, double goal_d, double goal_d_dot, vector<double> &s_coeffs, vector<double> &d_coeffs);
  void realize_behavior(SDVehicle& sdcar, const vector<double> &s_coeff, const vector<double> &d_coeff);
  
  void find_closest_cars_inlane(double ego_s,
      const vector<deque<Vehicle> >& inlane_veh_trajectories,
      deque<Vehicle>& res_frontcar,
      deque<Vehicle>& res_rearcar);
  double calc_behaviorlane_cost(SDVehicle& sdcar, vector<deque<Vehicle> >& inlane_veh_trajectories);
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
  virtual void entry(SDVehicle& sdcar);
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
