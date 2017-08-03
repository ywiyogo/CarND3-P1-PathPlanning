#include "behaviorfsm.h"
#include "helper.h"
#include <iostream>

#define DEBUG 0

using namespace std;

const int horizon_t = 1;
const int PRED_TIME = 1;
const int MAX_VEL = 25;     // m/s
const int MAX_JERK = 10;    // in m/s3
const int MAX_ACC = 10;     // in m/s2
const int DIST_BUFFER = 3;  // in m
const double SIM_dT = 0.02; // 0.02s

double max_dist = MAX_VEL * PRED_TIME;
//-----------------------------------------
// Abstract Class
//-----------------------------------------

BehaviorFSM::BehaviorFSM()
{
}
BehaviorFSM::BehaviorFSM(const string& name)
{
  this->name_ = name;
  cout << "Enter " << this->name_ << endl;
}

BehaviorFSM::~BehaviorFSM()
{
  cout << "Deleted " << this->name_ << endl;
}

void BehaviorFSM::set_behavior_state(SDVehicle& sdcar, BehaviorFSM* state)
{
  BehaviorFSM* aux = sdcar.behaviorfsm_;
  sdcar.behaviorfsm_ = state;
  delete aux;
}

// Base Implementation
void BehaviorFSM::update_ego(SDVehicle& sdcar,
    EgoVehicle& ego,
    const vector<double>& prev_path_x,
    const vector<double>& prev_path_y)
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
  if(sdcar.acc_list.size() >= 10) {
    sdcar.acc_list.pop_front();
  }
  sdcar.jerk = jerk;
  cout << "x: " << sdcar.x << " y: " << sdcar.y << " s: " << sdcar.s << " d: " << sdcar.d << " yaw: " << sdcar.yaw
       << " speed: " << sdcar.v_ms << endl;
  cout << "a: " << sdcar.a << " jerk: " << sdcar.jerk << endl;
  cout << "--------------------------------------------" << endl;
  // Clear the waypoints to prevent lag and appending the old one
  sdcar.next_x_vals.clear();
  sdcar.next_y_vals.clear();
}

void BehaviorFSM::realize_behavior(SDVehicle& sdcar, const vector<double>& s_coeff, const vector<double>& d_coeff)
{
  vector<double> XY;
  // cout << "s coeffs: ";
  for(int i = 0; i < s_coeff.size(); i++) {
    // cout << s_coeff[i] << " ";
  }
  // cout << endl;
  double t = 0.;

  while(t < PRED_TIME) {
    double s_t = s_coeff[0] + s_coeff[1] * t + s_coeff[2] * t * t + s_coeff[3] * t * t * t +
        s_coeff[4] * t * t * t * t + s_coeff[5] * t * t * t * t * t;
    if(!d_coeff.empty()) {
      double d_t = d_coeff[0] + d_coeff[1] * t + d_coeff[2] * t * t + d_coeff[3] * t * t * t +
          d_coeff[4] * t * t * t * t + d_coeff[5] * t * t * t * t * t;
      XY = sdcar.getXY(s_t, d_t);
    } else {
      XY = sdcar.getXY(s_t, sdcar.d);
    }

#if 0
    cout << "t,s: (" << t << ", " << s_t << ") ";
    cout << " x,y: (" << XY[0] << ", " << XY[1] << ")" << endl;
#endif
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
  cout << "Enter state: " << this->name_ << endl;
}

/*
 * ego_car(s, d, v)
 * pred_vehicles[id, x, y, vx,vy, s,d]
 */
double BehaviorFSM::calc_behaviorlane_cost(SDVehicle& sdcar, vector<deque<Vehicle> >& inlane_veh_trajectories)
{
  double totalcost;

  deque<Vehicle> frontcar;
  deque<Vehicle> rearcar;
  if(inlane_veh_trajectories.size() == 0) {
    return 0.;
  }
  find_closest_cars_inlane(sdcar.s, inlane_veh_trajectories, frontcar, rearcar);
  cout << "Debug calc cost" << endl;
  // front car has more cost than rear car
  if(frontcar.size() == 0) {
    totalcost += 0;
  } else if(frontcar.back().v_ms > sdcar.v_ms) {
    cout << "Front car speed is higher than self" << endl;
    totalcost += 1;
  } else if(frontcar.back().s - sdcar.s > DIST_BUFFER) {
    cout << "Front car has enough distance" << endl;
    totalcost += 2;
  } else {
    cout << "Front car distance too close!" << endl;
    totalcost += 3;
  }

  if(rearcar.size() == 0) {
    totalcost += 0;
  } else if(rearcar.back().v_ms < sdcar.v_ms) {
    totalcost += 0.5;
  } else if(rearcar.back().s - sdcar.s > DIST_BUFFER) {
    cout << "Rear car has enough distance" << endl;
    totalcost += 1;
  } else {
    cout << "Rear car distance too close!" << endl;
    totalcost += 1.5;
  }

  return totalcost;
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
    const vector<deque<Vehicle> >& inlane_veh_trajectories,
    deque<Vehicle>& res_frontcar,
    deque<Vehicle>& res_rearcar)
{
  double front_min_dist = 999;
  double rear_min_dist = -999;
  int front_id, rear_id = -1;
  int car_amount = inlane_veh_trajectories.size();
  for(int i = 0; i < car_amount; i++) {
    int trajectory_size = inlane_veh_trajectories[i].size();
    double curr_s = inlane_veh_trajectories[i][trajectory_size - 2].s;
    double diff = curr_s - ego_s;
    if(diff > 0) {
      if(diff < front_min_dist) {
        front_min_dist = diff;

        front_id = i;
      }
    } else {
      if(diff > rear_min_dist) {
        rear_min_dist = diff;

        rear_id = i;
      }
    }
  }
  if(front_id > -1)
    res_frontcar = inlane_veh_trajectories[front_id];
  if(rear_id > -1)
    res_rearcar = inlane_veh_trajectories[rear_id];
  return;
}

//--------------------------------------------------------
// Behaviour State Definitions
//--------------------------------------------------------

Ready::~Ready()
{
}

void Ready::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  if(sdcar.v_ms > 0) {

    set_behavior_state(sdcar, new KeepLane("KeepLane"));

  } else {
    vector<deque<Vehicle> > lane_trajectories;
    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == 0) {
        lane_trajectories.push_back(iter_veh.second);
      }
    }

    double cost = calc_behaviorlane_cost(sdcar, lane_trajectories);
    cout << "Cost in Ready: " << cost << endl;
    if(cost < 4) { // if car in front far away OR no car in front
      double end_s = sdcar.s + max_dist;
      vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
      vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
      vector<double> d_coeff;
      // d does not change
      set_behavior_state(sdcar, new KeepLane("KeepLane"));

      realize_behavior(sdcar, s_coeff, d_coeff);
    } else {
      // stop
    }
  }
}

//---------------------------------------------------------------------------------

KeepLane::~KeepLane()
{
}
void KeepLane::entry(SDVehicle& sdcar)
{
  //  cout << "Enter" << this->name_ << endl;
  //  double end_s = sdcar.s + max_dist;
  //  vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  //  vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
  //  vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
  //  realize_behavior(sdcar, s_coeff);
}
void KeepLane::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
  this->lane_ = get_lane(sdcar.d);
  vector<double> s_coeffs, start_s, goal_s;
  // 1. check the possible lane
  switch(this->lane_) {
  //------------------------
  // Lane 0
  //------------------------
  case 0: {
    cout << "DEbug line0" << endl;
    vector<deque<Vehicle> > lane_trajectories_0;
    vector<deque<Vehicle> > lane_trajectories_1;
    map<int, double> costs;
    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == 0) {
        lane_trajectories_0.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane

    costs[0] = calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);

    // 3. generate straight trajectory
    // there is no trajectory different if the cost of lane 1 is the minimum
    double end_s = sdcar.s + max_dist;
    vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
    vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
    s_coeffs = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
    vector<double> d_coeff;
    realize_behavior(sdcar, s_coeffs, d_coeff);
    // 4. set new state if the cost says to move
    if(costs[1] < costs[0]) {
#ifdef DEBUG
      cout << "Min Cost is on lane: 1"
           << " val: " << costs[1] << endl;
#endif
      set_behavior_state(sdcar, new PrepareLCR("PrepareLCR"));
    }

    break;
  }
  //------------------------
  // Lane 1
  //------------------------
  case 1: {
    vector<deque<Vehicle> > lane_trajectories_0;
    vector<deque<Vehicle> > lane_trajectories_1;
    vector<deque<Vehicle> > lane_trajectories_2;
    map<int, double> costs;

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == 0) {
        lane_trajectories_0.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == 2) {
        lane_trajectories_2.push_back(iter_veh.second);
      } else {
        cout << "Invalid lane found!!" << endl;
      }
    }
    cout << "Debug0" << endl;
    // 2. calculate the cost for each possible lane, add 0.1 and 0.2 for change lane
    costs[0] = 0.1 + calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);

    costs[2] = 0.2 + calc_behaviorlane_cost(sdcar, lane_trajectories_2);
    cout << "Cost lane 0: " << costs[0] << " lane 1: " << costs[1] << " lane 2: " << costs[2] << endl;
    vector<double> mincost = { -1, 9 }; //(lane, cost)
    cout << "Debug1" << endl;
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }
#ifdef DEBUG
    cout << "Min Cost is on lane: " << mincost[0] << " val: " << mincost[1] << endl;
#endif
    // 3. generate straight trajectory
    double sg_dot, sg_dotdot;
    if(mincost[1] < 2) {
      sg_dot = sdcar.s_dot;
      sg_dotdot = sdcar.s_dotdot;
    } else {
      sg_dot = sdcar.s_dot / 2;
      sg_dotdot = sdcar.s_dotdot / 2;
    }
    double end_s = sdcar.s + max_dist;
    vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
    vector<double> goal_s = { end_s, sg_dot, sg_dotdot };
    vector<double> d_coeffs;
    s_coeffs = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);

    realize_behavior(sdcar, s_coeffs, d_coeffs);

    // 4. set new state if the cost says to move
    switch(int(mincost[0])) {
    case 0:

      set_behavior_state(sdcar, new PrepareLCL("PrepareLCL"));
      break;

    case 2:
      set_behavior_state(sdcar, new PrepareLCR("PrepareLCR"));
      break;
    default:
#ifdef DEBUG
      cout << "Keep lane..." << endl;
#endif
    }
    break;
  }

  case 2: {
    vector<deque<Vehicle> > lane_trajectories_1;
    vector<deque<Vehicle> > lane_trajectories_2;

    map<int, double> costs; //(lane, cost)

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == 2) {
        lane_trajectories_2.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);
    costs[2] = calc_behaviorlane_cost(sdcar, lane_trajectories_2);

    // 3. generate straight trajectory
    // there is no trajectory different if the cost of lane 1 is the minimum
    double end_s = sdcar.s + max_dist;
    vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
    vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
    s_coeffs = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
    vector<double> d_coeffs;
    realize_behavior(sdcar, s_coeffs, d_coeffs);

    // 4. set new state if the cost says to move
    if(costs[1] < costs[2]) {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[1] << endl;
#endif
      set_behavior_state(sdcar, new PrepareLCL("PrepareLCL"));
    }
    break;
  }
  }

  //  vector<deque<Vehicle> > front_rear_cars;
  //  deque<Vehicle> frontcar;
  //  frontcar.push_back(Vehicle());
  //  front_rear_cars.push_back(frontcar);
  //  deque<Vehicle> rearcar;
  //  frontcar.push_back(Vehicle());
  //  front_rear_cars.push_back(rearcar);
  //  find_closest_cars_inlane(sdcar.s, inlane_trajectories, front_rear_cars);
  //
  //  if(front_rear_cars[0][0].id != -1) {
  //    if(fabs(sdcar.s - front_rear_cars[0].back().s) >= BUFFER) {
  //      cout << "Car found but far away" << endl;
  //      double end_s = sdcar.s + max_dist;
  //      vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  //      vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
  //      vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
  //      // d does not change
  //      realize_behavior(sdcar, s_coeff);
  //    }else{
  //      cout << "A slower car found" << endl;
  //      double end_s = sdcar.s + max_dist;
  //      vector<double> start_s = { sdcar.s, sdcar.v_ms, sdcar.a};
  //      vector<double> goal_s = { end_s, sdcar.s_dot/2., sdcar.s_dotdot/2. };
  //      vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
  //    }
  //  } else {
  //    cout << "No front car found" << endl;
  //    double end_s = sdcar.s + max_dist;
  //      vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  //      vector<double> goal_s = { end_s, sdcar.s_dot, sdcar.s_dotdot };
  //    vector<double> s_coeff = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);
  //    // d does not change
  //
  //    realize_behavior(sdcar, s_coeff);
  //  }
}
//---------------------------------------------------------------------------------
// LCL::LCL()
//{
//  this->name_ = "LCL";
//  cout << "Enter " << this->name_ << endl;
//}

LCL::~LCL()
{
}

void LCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
  this->lane_ = get_lane(sdcar.d);
  vector<double> s_coeffs, start_s, goal_s;
  // 1. check the possible lane
}

//---------------------------------------------------------------------------------

PrepareLCL::~PrepareLCL()
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
}

void PrepareLCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
  this->lane_ = get_lane(sdcar.d);
  vector<double> s_coeffs, start_s, goal_s;
  // 1. check the possible lane
  switch(this->lane_) {
  //------------------------
  // Lane 0
  //------------------------
  case 0: {
  }
  case 1: {
  }
  case 2: {
  }
  }
}

//---------------------------------------------------------------------------------

LCR::~LCR()
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
}

void LCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
  
  this->lane_ = get_lane(sdcar.d);
  vector<double> s_coeffs, d_coeffs, start_s, goal_s, start_d, goal_d;

  // 1. check the possible lane
  switch(this->lane_) {
  //------------------------
  // Lane 0
  //------------------------
  case 1: {
    vector<deque<Vehicle> > lane_trajectories_0;
    vector<deque<Vehicle> > lane_trajectories_1;
    vector<deque<Vehicle> > lane_trajectories_2;
    map<int, double> costs;

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == 0) {
        lane_trajectories_0.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == 2) {
        lane_trajectories_2.push_back(iter_veh.second);
      } else {
        cout << "Invalid lane found!!" << endl;
      }
    }
    cout << "Debug0" << endl;
    // 2. calculate the cost for each possible lane, add 0.1 and 0.2 for change lane
    costs[0] = 0.1 + calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);

    costs[2] = 0.2 + calc_behaviorlane_cost(sdcar, lane_trajectories_2);
    cout << "Cost lane 0: " << costs[0] << " lane 1: " << costs[1] << " lane 2: " << costs[2] << endl;
    vector<double> mincost = { -1, 9 }; //(lane, cost)
    cout << "Debug1" << endl;
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }
#ifdef DEBUG
    cout << "Min Cost is on lane: " << mincost[0] << " val: " << mincost[1] << endl;
#endif
    // 3. generate straight trajectory
    double sg_dot, sg_dotdot;
    if(mincost[1] < 2) {
      sg_dot = sdcar.s_dot;
      sg_dotdot = sdcar.s_dotdot;
    } else {
      sg_dot = sdcar.s_dot / 2;
      sg_dotdot = sdcar.s_dotdot / 2;
    }
    double end_s = sdcar.s + max_dist;
    vector<double> start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
    vector<double> goal_s = { end_s, sg_dot, sg_dotdot };
    vector<double> d_coeffs;
    s_coeffs = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);

    realize_behavior(sdcar, s_coeffs, d_coeffs);

    // 4. set new state if the cost says to move
    if(mincost[0] == 2) {
      set_behavior_state(sdcar, new KeepLane("KeepLane"));
    }

#ifdef DEBUG
    cout << "Keep lane..." << endl;
#endif
  } 

case 2: {
  vector<deque<Vehicle> > lane_trajectories_1;
  vector<deque<Vehicle> > lane_trajectories_2;

  map<int, double> costs;             //(lane, cost)
  vector<double> mincost = { -1, 9 }; //(lane, cost)

  for(const auto& iter_veh : cars_trajectories) {
    if(get_lane(iter_veh.second.back().d) == sdcar.d) {
      lane_trajectories_1.push_back(iter_veh.second);
    } else if(get_lane(iter_veh.second.back().d) == sdcar.d + 1) {
      lane_trajectories_2.push_back(iter_veh.second);
    }
  }
  // 2. calculate the cost for each possible lane
  costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);
  costs[2] = calc_behaviorlane_cost(sdcar, lane_trajectories_2);

  // 3. generate spline trajectory
  // there is no trajectory different if the cost of lane 1 is the minimum
  double sg_dot, sg_dotdot;
  if(mincost[1] < 2) {
    sg_dot = sdcar.s_dot;
    sg_dotdot = sdcar.s_dotdot;

  } else {
    sg_dot = sdcar.s_dot / 2;
    sg_dotdot = sdcar.s_dotdot / 2;

    double end_d = sdcar.d + 4;
    start_d = { sdcar.d, sdcar.d_dot, sdcar.d_dotdot };
    goal_d = { end_d, sdcar.d_dot / 2, sdcar.d_dotdot / 2 };
    d_coeffs = sdcar.jerk_min_trajectory(start_d, goal_d, PRED_TIME);
  }

  double end_s = sdcar.s + max_dist;
  start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  goal_s = { end_s, sg_dot, sg_dotdot };
  s_coeffs = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);

  realize_behavior(sdcar, s_coeffs, d_coeffs);

  // 4. set new state if the cost says to move
  if(costs[1] < costs[2]) {
#ifdef DEBUG
    cout << "Min Cost is on lane 1 "
         << " val: " << costs[1] << endl;
#endif
    set_behavior_state(sdcar, new KeepLane("KeepLane"));

  } else {
    set_behavior_state(sdcar, new PrepareLCL("LCR"));
  }
  break;
}
}
}
PrepareLCR::~PrepareLCR()
{
}
void PrepareLCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  this->lane_ = get_lane(sdcar.d);
  vector<double> s_coeffs, d_coeffs, start_s, goal_s, start_d, goal_d;

  vector<deque<Vehicle> > lane_trajectories_1;
  vector<deque<Vehicle> > lane_trajectories_2;

  map<int, double> costs;             //(lane, cost)
  vector<double> mincost = { -1, 9 }; //(lane, cost)

  for(const auto& iter_veh : cars_trajectories) {
    if(get_lane(iter_veh.second.back().d) == sdcar.d) {
      lane_trajectories_1.push_back(iter_veh.second);
    } else if(get_lane(iter_veh.second.back().d) == sdcar.d + 1) {
      lane_trajectories_2.push_back(iter_veh.second);
    }
  }
  // 2. calculate the cost for each possible lane
  costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);
  costs[2] = calc_behaviorlane_cost(sdcar, lane_trajectories_2);

  // 3. generate spline trajectory
  // there is no trajectory different if the cost of lane 1 is the minimum
  double sg_dot, sg_dotdot;
  if(mincost[1] < 2) {
    sg_dot = sdcar.s_dot;
    sg_dotdot = sdcar.s_dotdot;

  } else {
    sg_dot = sdcar.s_dot / 2;
    sg_dotdot = sdcar.s_dotdot / 2;

    double end_d = sdcar.d + 4;
    start_d = { sdcar.d, sdcar.d_dot, sdcar.d_dotdot };
    goal_d = { end_d, sdcar.d_dot / 2, sdcar.d_dotdot / 2 };
    d_coeffs = sdcar.jerk_min_trajectory(start_d, goal_d, PRED_TIME);
  }

  double end_s = sdcar.s + max_dist;
  start_s = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  goal_s = { end_s, sg_dot, sg_dotdot };
  s_coeffs = sdcar.jerk_min_trajectory(start_s, goal_s, PRED_TIME);

  realize_behavior(sdcar, s_coeffs, d_coeffs);

  // 4. set new state if the cost says to move
  if(costs[1] < costs[2]) {
#ifdef DEBUG
    cout << "Min Cost is on lane 1 "
         << " val: " << costs[1] << endl;
#endif
    set_behavior_state(sdcar, new KeepLane("KeepLane"));

  } else if(sdcar.d < this->lane_ * 4 + 2) {
    set_behavior_state(sdcar, new PrepareLCL("PrepareLCR"));
  } else {
    set_behavior_state(sdcar, new LCR("LCR"));
  }
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
