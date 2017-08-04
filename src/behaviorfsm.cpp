#include "behaviorfsm.h"
#include "helper.h"
#include <iostream>

#define DEBUG 0

using namespace std;
using namespace Helper;

const int horizon_t = 1;
const int PRED_TIME = 1;
const int MAX_VEL = 25;     // m/s
const int MAX_JERK = 10;    // in m/s3
const int MAX_ACC = 10;     // in m/s2
const int DIST_BUFFER = 7;  // in m
const double SIM_dT = 0.02; // 0.02s

double max_dist = MAX_VEL * PRED_TIME;
//-----------------------------------------
// Abstract Class
//-----------------------------------------

BehaviorFSM::BehaviorFSM()
{
}
BehaviorFSM::BehaviorFSM(const string& name, int lane)
{
  this->name_ = name;
  this->goallane_ = lane;
  cout << "##########################################################" << endl;
  cout << "Enter " << this->name_ << " goal lane is " << lane << endl;
  cout << "##########################################################" << endl;
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
  if(cur_s_dot > 10) {
    cur_s_dot = 0.;
  }
  sdcar.s_dotdot = cur_s_dot - sdcar.s_dot;
  sdcar.s_dot = cur_s_dot;

  double cur_d_dot = ego.d - sdcar.d;
  if(cur_d_dot > 10) {
    cur_d_dot = 0.;
  }
  sdcar.d_dotdot = cur_d_dot - sdcar.d_dot;
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
  if(sdcar.acc_list.size() > 4)
    sdcar.jerk = jerk;
  cout << "x: " << sdcar.x << " y: " << sdcar.y << " s: " << sdcar.s << " d: " << sdcar.d << " yaw: " << sdcar.yaw
       << " speed: " << sdcar.v_ms << endl;
  cout << "s_dot: " << sdcar.s_dot << " s_dotdot: " << sdcar.s_dotdot << " a: " << sdcar.a << " jerk: " << sdcar.jerk
       << endl;
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

  int size = frontcar.size();
  cout << "Lane " << get_lane(frontcar[size - 2].d) << endl;
  // front car has more cost than rear car
  if(frontcar.size() == 0) {
    totalcost += 0;
  } else if(frontcar[size - 2].v_ms > sdcar.v_ms) {
    cout << "Front car " << frontcar[size - 2].id << " speed is higher than self" << endl;
    totalcost += 1;
  } else if(frontcar[size - 2].s - sdcar.s > DIST_BUFFER) {
    cout << "Front car " << frontcar[size - 2].id << " has enough distance:" << frontcar.back().s - sdcar.s << endl;
    totalcost += 2;
  } else {
    cout << "Front car " << frontcar[size - 2].id << " distance too close!" << endl;
    totalcost += 3;
  }
  size = rearcar.size();
  if(rearcar.size() == 0) {
    totalcost += 0;
  } else if(rearcar[size - 2].v_ms < sdcar.v_ms) {
    totalcost += 0.1;
  } else if(rearcar[size - 2].s - sdcar.s > DIST_BUFFER) {
    cout << "Rear car " << frontcar[size - 2].id << " has enough distance" << endl;
    totalcost += 0.2;
  } else {
    cout << "Rear car distance too close!" << endl;
    totalcost += 0.3;
  }

  return totalcost;
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

void BehaviorFSM::generate_trajectory(SDVehicle& sdcar,
    double goal_s_dot,
    double goal_s_dotdot,
    double goal_d,
    double goal_d_dot,
    vector<double>& s_coeffs,
    vector<double>& d_coeffs)
{
  cout << "Generate trajectory" << endl;
  if(s_coeffs.size() != 0)
    s_coeffs.clear();
  if(d_coeffs.size() != 0)
    d_coeffs.clear();
  double max_s_dotdot = 0.001;
  double max_s_dot = 0.01;
  //  if(goal_s_dotdot > max_s_dotdot)
  //  {
  //    cout<<"s_dotdot is too big, cut to "<< max_s_dotdot<<endl;
  //    goal_s_dotdot = max_s_dotdot; // sdcar.s_dotdot;
  //  }
  //  if(goal_s_dot > max_s_dot)
  //  {
  //    cout<<"s_dot is too big, cut to "<<max_s_dot<<endl;
  //    goal_s_dot = max_s_dot; // sdcar.s_dotdot;
  //  }
  vector<double> sstart, sgoal, dstart, dgoal;
  double goal_s;

  sstart = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };

  goal_s = sdcar.s + max_dist;

  sgoal = { goal_s, max_s_dot, max_s_dotdot };
  s_coeffs = sdcar.jerk_min_trajectory(sstart, sgoal, PRED_TIME);

  dstart = { sdcar.d, sdcar.d_dot, sdcar.d_dotdot };
  dgoal = { goal_d, goal_d_dot, sdcar.d_dotdot };

  d_coeffs = sdcar.jerk_min_trajectory(dstart, dgoal, PRED_TIME);
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

    set_behavior_state(sdcar, new KeepLane("KeepLane", get_lane(sdcar.d)));

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

      vector<double> s_coeffs, d_coeffs;
      generate_trajectory(sdcar, sdcar.s_dot, sdcar.s_dotdot, sdcar.d, sdcar.d_dot, s_coeffs, d_coeffs);
      // d does not change
      set_behavior_state(sdcar, new KeepLane("KeepLane", get_lane(sdcar.d)));

      realize_behavior(sdcar, s_coeffs, d_coeffs);
    } else {
      // stop
      cout << "Car cannot start due to high cost in current lane" << endl;
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
  int currlane = get_lane(sdcar.d);
  vector<double> s_coeffs, start_s, goal_s;
  map<int, double> costs; //(lane, cost)
  vector<deque<Vehicle> > lane_trajectories_0, lane_trajectories_1, lane_trajectories_2;
  // 1. check the possible lane
  switch(currlane) {
  //------------------------
  // Lane 0
  //------------------------
  case 0: {
    cout << "Debug line 0" << endl;
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

    vector<double> s_coeffs, d_coeffs;
    generate_trajectory(sdcar, sdcar.s_dot, sdcar.s_dotdot, sdcar.d, sdcar.d_dot, s_coeffs, d_coeffs);

    realize_behavior(sdcar, s_coeffs, d_coeffs);
    // 4. set new state if the cost says to move
    if(costs[1] < costs[0]) {
#ifdef DEBUG
      cout << "Min Cost is on lane: 1"
           << " val: " << costs[1] << endl;
#endif
      set_behavior_state(sdcar, new PrepareLCR("PrepareLCR", 1));
    }

    break;
  }
  //------------------------
  // Lane 1
  //------------------------
  case 1: {
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
    // 2. calculate the cost for each possible lane, add 0.1 and 0.2 for change lane
    costs[0] = 0.1 + calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);

    costs[2] = 0.2 + calc_behaviorlane_cost(sdcar, lane_trajectories_2);
    cout << "Cost lane 0: " << costs[0] << " lane 1: " << costs[1] << " lane 2: " << costs[2] << endl;
    vector<double> mincost = { -1, 9 }; //(lane, cost)

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
    vector<double> s_coeffs, d_coeffs;
    if(mincost[1] > 3) {
      sdcar.d_dotdot = -0.01;
      cout << "Break" << endl;
    }
    sdcar.d_dotdot = 0.01;
    generate_trajectory(sdcar, sdcar.s_dot, sdcar.s_dotdot, sdcar.d, sdcar.d_dot, s_coeffs, d_coeffs);

    realize_behavior(sdcar, s_coeffs, d_coeffs);

    // 4. set new state if the cost says to move
    switch(int(mincost[0])) {
    case 0:

      set_behavior_state(sdcar, new PrepareLCL("PrepareLCL", 1));
      break;
    case 2:
      set_behavior_state(sdcar, new PrepareLCR("PrepareLCR", 1));
      break;
    default:
#ifdef DEBUG
      cout << "Keep lane..." << endl;
#endif
    }
    break;
  }

  case 2: {
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
    vector<double> s_coeffs, d_coeffs;
    generate_trajectory(sdcar, sdcar.s_dot, sdcar.s_dotdot, sdcar.d, sdcar.d_dot, s_coeffs, d_coeffs);

    realize_behavior(sdcar, s_coeffs, d_coeffs);
    cout << "  Cost lane 1 " << costs[1] << " lane 2: " << costs[2] << endl;
    // 4. set new state if the cost says to move
    if(costs[1] < costs[2]) {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[1] << endl;
#endif
      set_behavior_state(sdcar, new PrepareLCL("PrepareLCL", 1));
    } else {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[1] << endl;
#endif
    }
    break;
  }
  }
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
  // In LCL state the robot will drive to left. In this case there are 2 possibilities:
  // from lane 1 to 0 and from lane 2 to 1. During the transitions the robot will drive until
  // the d value of the goal lane is reached. If this is reach then the robot can keep on the lane
  int currlane = get_lane(sdcar.d);
  vector<double> s_coeffs, d_coeffs;

  // 1. check the possible lane
  if(0 < sdcar.d && sdcar.d < 6) { // Lane 1->0
    if(fabs(sdcar.d - 2) > 0.2) {  // goal not yet reached, generate trajectory
      // 3. generate  trajectoryto lane 0

      generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot, 2, sdcar.d_dot, s_coeffs, d_coeffs);
      realize_behavior(sdcar, s_coeffs, d_coeffs);

    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }

  } else if(6 < sdcar.d && sdcar.d < 12) { // Lane 2 -> 1
    if(fabs(sdcar.d - 6) > 0.2) {          // goal not yet reached, generate trajectory
      // 3. generate  trajectory to lane 1
      generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot, 6, sdcar.d_dot, s_coeffs, d_coeffs);
      realize_behavior(sdcar, s_coeffs, d_coeffs);

    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
  } else {
    cout << "Invalid d" << endl;
  }
}

//---------------------------------------------------------------------------------

PrepareLCL::~PrepareLCL()
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
}

void PrepareLCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  // Prepare LCL state can only be perform if the robot drives on lane 1 or lane 2
  int currlane = get_lane(sdcar.d);
  vector<double> s_coeffs, d_coeffs, start_s, goal_s;

  switch(currlane) {
  case 1: { // prepare 1 -> 0
    vector<deque<Vehicle> > lane_trajectories_0;
    vector<deque<Vehicle> > lane_trajectories_1;
    map<int, double> costs;             //(lane, cost)
    vector<double> mincost = { -1, 9 }; //(lane, cost)

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == currlane) {
        lane_trajectories_0.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == currlane - 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane
    costs[0] = calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);

    // whatever the mincost is the robot still drive forward with less vel and acc
    generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot / 2, sdcar.d, sdcar.d_dot, s_coeffs, d_coeffs);
    realize_behavior(sdcar, s_coeffs, d_coeffs);

    if(costs[0] < costs[1]) {
#ifdef DEBUG
      cout << "Min Cost is on lane 0 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new LCL("LCL", currlane - 1));
    } else {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[1] << endl;
#endif
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }

  } break;
  //--------------
  // Robot on lane 2 goal lane 1
  //--------------
  case 2: {

    vector<deque<Vehicle> > lane_trajectories_1;
    vector<deque<Vehicle> > lane_trajectories_2;
    map<int, double> costs;             //(lane, cost)
    vector<double> mincost = { -1, 9 }; //(lane, cost)

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == currlane) {
        lane_trajectories_2.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == currlane - 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);
    costs[2] = calc_behaviorlane_cost(sdcar, lane_trajectories_2);

    // whatever the mincost is the robot still drive forward, but with slower velocity and acc
    generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot, sdcar.d, sdcar.d_dot / 2., s_coeffs, d_coeffs);
    realize_behavior(sdcar, s_coeffs, d_coeffs);

    if(costs[1] < costs[2]) {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new LCL("LCL", currlane - 1));
    } else {
      cout << "Min Cost is on lane 2"
           << " val: " << costs[0] << endl;
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }

  } break;

  default:
    cout << "Invalid lane in lane : " << currlane << endl;
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
  // 1. check the possible lane
  int currlane = get_lane(sdcar.d);
  vector<double> s_coeffs, d_coeffs;

  if(0 < sdcar.d && sdcar.d < 6) { // Lane 0 -> 1
    if(fabs(sdcar.d - 6) > 0.2) {  // goal not yet reached, generate trajectory
      generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot, 4, sdcar.d_dot, s_coeffs, d_coeffs);
      realize_behavior(sdcar, s_coeffs, d_coeffs);
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }

  } else if(6 < sdcar.d && sdcar.d < 12) { // Lane 1 -> 2
    if(fabs(sdcar.d - 10) > 0.2) {         // goal not yet reached, generate trajectory
      generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot, 10, sdcar.d_dot, s_coeffs, d_coeffs);
      realize_behavior(sdcar, s_coeffs, d_coeffs);
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
  } else {
    cout << "Invalid d" << endl;
  }
}
PrepareLCR::~PrepareLCR()
{
}
void PrepareLCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  int currlane = get_lane(sdcar.d);
  vector<double> s_coeffs, d_coeffs, start_s, goal_s, start_d, goal_d;

  switch(currlane) {
  // Robot on lane 0 praparing to lane 1
  case 0: {
    vector<deque<Vehicle> > lane_trajectories_0;
    vector<deque<Vehicle> > lane_trajectories_1;
    map<int, double> costs;             //(lane, cost)
    vector<double> mincost = { -1, 9 }; //(lane, cost)

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == currlane) {
        lane_trajectories_0.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == currlane + 1) {
        lane_trajectories_1.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane
    costs[0] = calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);

    // whatever the mincost is the robot still drive forward
    generate_trajectory(sdcar, sdcar.s_dot, sdcar.s_dotdot, sdcar.d, sdcar.d_dot, s_coeffs, d_coeffs);
    realize_behavior(sdcar, s_coeffs, d_coeffs);

    if(costs[0] < costs[1]) {
#ifdef DEBUG
      cout << "Min Cost is on lane 0 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));

    } else {
      set_behavior_state(sdcar, new LCR("LCR", currlane + 1));
    }

  } break;
  //--------------
  // Robot on lane 1->2
  //--------------
  case 1: {

    vector<deque<Vehicle> > lane_trajectories_1;
    vector<deque<Vehicle> > lane_trajectories_2;
    map<int, double> costs;             //(lane, cost)
    vector<double> mincost = { -1, 9 }; //(lane, cost)

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == currlane) {
        lane_trajectories_1.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == currlane + 1) {
        lane_trajectories_2.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);
    costs[2] = calc_behaviorlane_cost(sdcar, lane_trajectories_2);

    // whatever the mincost is the robot still drive forward, but with slower velocity and acc
    generate_trajectory(sdcar, sdcar.s_dot / 2, sdcar.s_dotdot, sdcar.d, sdcar.d_dot / 2., s_coeffs, d_coeffs);
    realize_behavior(sdcar, s_coeffs, d_coeffs);

    if(costs[1] < costs[2]) {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));

    } else {
      set_behavior_state(sdcar, new LCR("LCR", currlane + 1));
    }

  } break;

  default:
    cout << "Invalid lane in lane : " << currlane << endl;
  }
}
