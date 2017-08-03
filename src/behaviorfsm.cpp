#include "behaviorfsm.h"
#include "helper.h"
#include <iostream>

#define DEBUG_LVL 2 //0: nodebug, 1 only error, 2: critical info and warn 3: all

using namespace std;

const int horizon_t = 1;
const int PRED_TIME = 1;
const int MAX_VEL = 25; //m/s
const int MAX_JERK = 10;  //in m/s3
const int MAX_ACC = 10; // in m/s2
const int BUFFER = 4;   // in m
const double SIM_dT= 0.02; //0.02s

double max_dist = MAX_VEL * PRED_TIME;
//-----------------------------------------
// Abstract Class
//-----------------------------------------
BehaviorFSM::BehaviorFSM()
{
  counter = 0;
}
BehaviorFSM::~BehaviorFSM()
{
  cout << "Deleted " << this->name << endl;
}

void BehaviorFSM::set_behavior_state(SDVehicle& sdcar, BehaviorFSM* state)
{
  BehaviorFSM* aux = sdcar.behaviorfsm_;
  sdcar.behaviorfsm_ = state;
  delete aux;
}

// Base Implementation
void BehaviorFSM::update_ego(SDVehicle& sdcar, EgoVehicle& ego, const vector<double> &prev_path_x, const vector<double> &prev_path_y)
{
  // updated car states
  sdcar.x = ego.x;
  sdcar.y = ego.y;
  double cur_s_dot = ego.s - sdcar.s;
  sdcar.s_dotdot = cur_s_dot - sdcar.s_dot;
  sdcar.s_dot = cur_s_dot;
  double cur_d_dot = ego.d - sdcar.d;
  sdcar.s_dotdot = cur_d_dot - sdcar.d_dot;
  sdcar.d_dot = cur_d_dot;
  sdcar.v_ms = ego.v_ms;
  sdcar.s = ego.s;
  sdcar.d = ego.d;
  sdcar.a = (sdcar.v_ms - sdcar.prev_v) / 0.2;
  sdcar.prev_v = sdcar.v_ms;
  sdcar.yaw = ego.yaw;
  sdcar.acc_list.push_back(sdcar.a);
  double jerk = 0;

  for(int i = 0; i < sdcar.acc_list.size(); i++) {
    jerk += sdcar.acc_list[i];
  }
  jerk = jerk / sdcar.acc_list.size();
  if(sdcar.acc_list.size()>=10)
  {
    sdcar.acc_list.pop_front();
  }
  sdcar.jerk = jerk;
  cout << "x: " << sdcar.x << " y: " << sdcar.y << " s: " << sdcar.s << " d: " << sdcar.d << " yaw: " << sdcar.yaw
       << " speed: " << sdcar.v_ms << endl;
  cout << "a: " << sdcar.a << " jerk: " << sdcar.jerk << endl;
  cout << "--------------------------------------------" << endl;
  //Clear the waypoints to prevent lag and appending the old one
  sdcar.next_x_vals.clear();
  sdcar.next_y_vals.clear();
}

void BehaviorFSM::realize_behavior(SDVehicle& sdcar, vector<double> s_coeff)
{
    vector<double> XY;
    //cout << "s coeffs: ";
    for(int i = 0; i < s_coeff.size(); i++) {
      //cout << s_coeff[i] << " ";
    }
    //cout << endl;
    double t = 0.;

    while(t < PRED_TIME) {
      double s_t = s_coeff[0] + s_coeff[1] * t + s_coeff[2] * t * t + s_coeff[3] * t * t * t +
          s_coeff[4] * t * t * t * t + s_coeff[5] * t * t * t * t * t;
      cout << "t,s: (" << t << ", " << s_t<<") ";

      XY = sdcar.getXY(s_t, sdcar.d);
      cout << " x,y: (" << XY[0] << ", " << XY[1] <<")"<< endl;
      sdcar.next_x_vals.push_back(XY[0]);
      sdcar.next_y_vals.push_back(XY[1]);
      t += SIM_dT;
    }

    // Test
//    double pos_x;
//    double pos_y;
//    double angle;
//    int path_size = sdcar.prev_path_x.size();
//
//    for(int i = 0; i < path_size; i++) {
//      sdcar.next_x_vals.push_back(sdcar.prev_path_x[i]);
//      sdcar.next_y_vals.push_back(sdcar.prev_path_y[i]);
//    }
//
//    if(path_size == 0) {
//      pos_x = sdcar.x;
//      pos_y = sdcar.y;
//      angle = Helper::deg2rad(sdcar.yaw);
//    } else {
//      pos_x = sdcar.prev_path_x[path_size - 1];
//      pos_y = sdcar.prev_path_y[path_size - 1];
//
//      double pos_x2 = sdcar.prev_path_x[path_size - 2];
//      double pos_y2 = sdcar.prev_path_y[path_size - 2];
//      angle = atan2(pos_y - pos_y2, pos_x - pos_x2);
//    }
//
//    double dist_inc = 0.5;
//    for(int i = 0; i < 50 - path_size; i++) {
//      sdcar.next_x_vals.push_back(pos_x + (dist_inc)*cos(angle + (i + 1) * (M_PI / 100)));
//      sdcar.next_y_vals.push_back(pos_y + (dist_inc)*sin(angle + (i + 1) * (M_PI / 100)));
//      pos_x += (dist_inc)*cos(angle + (i + 1) * (M_PI / 100));
//      pos_y += (dist_inc)*sin(angle + (i + 1) * (M_PI / 100));
//    }
//    double dist_inc = 0.5;
//    for(int i = 0; i < 40; i++)
//    {
//          sdcar.next_x_vals.push_back(sdcar.x+(dist_inc*i)*cos(Helper::deg2rad(sdcar.yaw)));
//          sdcar.next_y_vals.push_back(sdcar.y+(dist_inc*i)*sin(Helper::deg2rad(sdcar.yaw)));
//    }
}

void BehaviorFSM::entry(SDVehicle& sdcar)
{
  cout << "Enter state: " << this->name << endl;
}

/*
 * ego_car(s, d, v)
 * pred_vehicles[id, x, y, vx,vy, s,d]
 */
double BehaviorFSM::calc_behaviorlane_cost(SDVehicle& sdcar, vector<deque<Vehicle>> &inlane_veh_trajectories)
{
  double total;
  double multiplier = 10;
  
  for(int i = 0; i < inlane_veh_trajectories.size(); i++) {
      double distance = fabs(inlane_veh_trajectories[i].back().s - sdcar.s);
      total += multiplier*distance;
  }
  return total;
}

/*
 * 0<d<4 is lane 0
   4<d<8 is lane 1
   8<d<12 is lane 2
 */
int BehaviorFSM::get_lane(double d)
{
  if(d < 4) {
    return 0;
  } else if(d < 8) {
    return 1;
  } else {
    return 2;
  }
}
// return first element is front vehicle trajectorie
// second element is rear vehicle trajectory
void BehaviorFSM::find_closest_cars_inlane(double ego_s,
    const map<int, deque<Vehicle> >& inlane_veh_trajectories,
    vector<deque<Vehicle> >& result)
{
  double front_min_dist = 999;
  double rear_min_dist = -999;
  if(result.size() != 2) {
    cout << "Error result != 2" << endl;
    return;
  }

  for(const auto& iter_veh : inlane_veh_trajectories) {
    double curr_s = iter_veh.second.at(iter_veh.second.size() - 2).s;
    double diff = curr_s - ego_s;
    if(diff > 0) {
      if(diff < front_min_dist) {
        front_min_dist = diff;

        result[0] = iter_veh.second;
      }
    } else {
      if(diff > rear_min_dist) {
        rear_min_dist = diff;

        result[1] = iter_veh.second;
      }
    }
  }

  return;
}


//--------------------------------------------------------
// Behaviour State Definitions
//--------------------------------------------------------
Ready::Ready()
{
  this->name = "Ready";
  cout << "Created " << this->name << endl;
}

Ready::~Ready()
{
}

void Ready::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  if(sdcar.v_ms > 0) {

    set_behavior_state(sdcar, new KeepLane());

  } else {
    map<int, deque<Vehicle> > inlane_trajectories;
    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == get_lane(sdcar.d)) {
        inlane_trajectories[iter_veh.first] = iter_veh.second;
      }
    }

    vector<deque<Vehicle> > front_rear_cars;
    deque<Vehicle> frontcar;
    frontcar.push_back(Vehicle());
    front_rear_cars.push_back(frontcar);
    deque<Vehicle> rearcar;
    frontcar.push_back(Vehicle());
    front_rear_cars.push_back(rearcar);
    find_closest_cars_inlane(sdcar.s, inlane_trajectories, front_rear_cars);

    

    if((front_rear_cars[0][0].id != -1 && fabs(sdcar.s - front_rear_cars[0].back().s) >= BUFFER) || 
        front_rear_cars[0][0].id == -1) {//if car in front far away OR no car in front 
      double end_s = sdcar.s + max_dist;
      vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
      // d does not change
      set_behavior_state(sdcar, new KeepLane());
      realize_behavior(sdcar, s_coeff);
    } else {
      // stop
    }
  }
}

//---------------------------------------------------------------------------------
KeepLane::KeepLane()
{
  this->name = "KeepLane";
  cout << "Created " << this->name << endl;
}
KeepLane::~KeepLane()
{
}
void KeepLane::entry(SDVehicle& sdcar)
{
  cout << "Enter" << this->name << endl;
  double end_s = sdcar.s + max_dist;
  vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
  vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
  realize_behavior(sdcar, s_coeff);
}
void KeepLane::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
//    possible_successor_states = successor_states(current_fsm_state)

  map<int, deque<Vehicle> > inlane_trajectories;
  for(const auto& iter_veh : cars_trajectories) {
    if(get_lane(iter_veh.second.back().d) == get_lane(sdcar.d)) {
      inlane_trajectories[iter_veh.first] = iter_veh.second;
    }
  }

  vector<deque<Vehicle> > front_rear_cars;
  deque<Vehicle> frontcar;
  frontcar.push_back(Vehicle());
  front_rear_cars.push_back(frontcar);
  deque<Vehicle> rearcar;
  frontcar.push_back(Vehicle());
  front_rear_cars.push_back(rearcar);
  find_closest_cars_inlane(sdcar.s, inlane_trajectories, front_rear_cars);
  
  if(front_rear_cars[0][0].id != -1) {
    if(fabs(sdcar.s - front_rear_cars[0].back().s) >= BUFFER) {
      cout << "Car found but far away" << endl;
      double end_s = sdcar.s + max_dist;
      vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
      // d does not change
      realize_behavior(sdcar, s_coeff);
    }else{
      cout << "A slower car found" << endl;
      double end_s = sdcar.s + max_dist;
      vector<double> start_s = { sdcar.s, sdcar.v_ms, sdcar.a};
      vector<double> goal_s = { end_s, sdcar.s_dot/2., sdcar.s_dotdot/2. };
      vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
    }
  } else {
    cout << "No front car found" << endl;
    double end_s = sdcar.s + max_dist;
      vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };;
    vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
    // d does not change

    realize_behavior(sdcar, s_coeff);
  }
}
//---------------------------------------------------------------------------------
LCL::LCL()
{
}

LCL::~LCL()
{
}

void LCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
//    possible_successor_states = successor_states(current_fsm_state)
}

//---------------------------------------------------------------------------------
PrepareLCL::PrepareLCL()
{}
PrepareLCL::~PrepareLCL()
{
  //    # only consider states which can be reached from current FSM state.
//    possible_successor_states = successor_states(current_fsm_state)
}

void PrepareLCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
//    possible_successor_states = successor_states(current_fsm_state)
}

//---------------------------------------------------------------------------------
LCR::LCR()
{
}
LCR::~LCR()
{
  //    # only consider states which can be reached from current FSM state.
//    possible_successor_states = successor_states(current_fsm_state)
}

void LCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
//    possible_successor_states = successor_states(current_fsm_state)
}

PrepareLCR::~PrepareLCR()
{
}

void PrepareLCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
}

// ----------------------------------------------------------------------------
// State: Keep Line
//

// class KL : public BehaviorFSM
//{
//  void entry() override
//  {
//    cout << "Entry KL" << endl;
//    //calculate trajectory
//    double ends = this->start_s[0] + this->start_s[1]*horizon_t;
//    vector<double> end_s ={ends, this->start_s[1], 0.}
//    vector<double> s_coeff = traj.jerk_min_trajectory(this->start_s, end_s , horizon_t);
//    vector<double> d_coeff = traj.jerk_min_trajectory(this->start_s, end_s , horizon_t);;
//    int steps = (double)horizon_t / 0.2;
//    vector<
//
//
//    }
//    s_t.push_back(value)
//    //realize trajectory
//
//  }
//
//  void react(MinCost const& e) override
//  {
//    cout << "Min cost: " << e.name << endl;
//
//    if(e.name.compare("PLCL") == 0) {
//      transit<PLCL>();
//    } else if(e.name.compare("PLCR") == 0) {
//      transit<PLCR>();
//    }
//  };
//};
//
//
