#include "behaviorfsm.h"
#include "helper.h"
#include <iostream>

#define DEBUG 0

using namespace std;
using namespace Helper;

const int horizon_t = 1;
const int PRED_TIME = 1;
const int MAX_VEL = 20;     // m/s
const int MAX_JERK = 10;    // in m/s3
const int MAX_ACC = 9;     // in m/s2
const int DIST_BUFFER = 30; // in m
const double V_BUFFER = 5;
const double SIM_dt = 0.02; // 0.02s
const double JMT_T = 2; //in s for JMT
const int STEPS = 50;
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
  this->suggest_acc_ = MAX_ACC;
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
    Vehicle& ego,
    const vector<double>& prev_path_x,
    const vector<double>& prev_path_y)
{
}


/*
 * ego_car(s, d, v)
 * pred_vehicles[id, x, y, vx,vy, s,d]
 */
double BehaviorFSM::calc_behaviorlane_cost(SDVehicle& sdcar, vector<deque<Vehicle> >& inlane_veh_trajectories)
{
  double totalcost = 0.;
  deque<Vehicle> frontcar;
  deque<Vehicle> rearcar;
  if(inlane_veh_trajectories.size() == 0) {
    return 0.;
  }
  find_closest_cars_inlane(sdcar.s, inlane_veh_trajectories, frontcar, rearcar);

  int size = frontcar.size();
  double distance = 0.0;
  // cout << "Lane " << get_lane(frontcar[size - 1].d) << endl;
  // front car has more cost than rear car
  if(frontcar.size() == 0) {
    totalcost += 0;
  } else {
    distance = fabs(frontcar[size - 1].s - sdcar.s);
    if(get_lane(sdcar.d) == get_lane(frontcar[size - 1].d)) {
      double a = distance / JMT_T / JMT_T;
      if(a < MAX_ACC) {
        this->suggest_acc_ = a;
      }
    }

    if(distance > DIST_BUFFER) {
      // cout << "Front car " << frontcar[size - 1].id << " has enough distance:" << distance << endl;
      if(distance < DIST_BUFFER + max_dist / 2) {
        totalcost += 0.2;
        if(frontcar[size - 1].v_ms > (sdcar.v_ms)) {
          // cout << "Front car " << frontcar[size - 1].id << " speed is higher than self" << endl;
          totalcost += 0;
        } else {
          totalcost += 0.2;
        }
      }
    } else {
      cout << "Front car " << frontcar[size - 1].id << " distance too close!" << endl;
      totalcost += 3;
    }
  }
  size = rearcar.size();
  if(size == 0) {
    totalcost += 0;
  } else {
    distance = fabs(rearcar[size - 1].s - sdcar.s);
    if(distance > DIST_BUFFER) {
      // cout << "Rear car " << frontcar[size - 1].id << " has enough distance: " << distance << endl;
      if(distance < DIST_BUFFER + max_dist / 2) {
        totalcost += 0.2;
      }
      if(rearcar[size - 1].v_ms < (sdcar.v_ms)) {
        totalcost += 0.2;
      }
    } else {
      cout << "Rear car distance too close!" << endl;
      totalcost += 0.3;
    }
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
    double acc,
    double goal_d,
    vector<double>& s_coeffs,
    vector<double>& d_coeffs,
    double T)
{
  cout << "Generate trajectory" << endl;
  if(s_coeffs.size() != 0)
    s_coeffs.clear();
  if(d_coeffs.size() != 0)
    d_coeffs.clear();

  vector<double> sstart, sgoal, dstart, dgoal;
  double goal_s;
//  if(sdcar.sim_delay>T){
//    T = sdcar.sim_delay;
//  }

  sstart = { sdcar.s, sdcar.s_dot, sdcar.s_dotdot };
  double ds = acc*T*T;
  if(ds > MAX_VEL)
  {
    ds = MAX_VEL; //max distance is in second ~ MAX_VELOCITY
  }
    
  goal_s = sdcar.s + ds;
  
  double goal_sdot = min(acc*sdcar.sim_delay, double(MAX_VEL));
  printf("given acc: %f, acc*T: %f, v: %f\n", acc, acc*sdcar.sim_delay, goal_sdot);
  double goal_sdotdot = min(acc, double(MAX_ACC));


  sgoal = { goal_s, goal_sdot, 0. };
  s_coeffs = sdcar.jerk_min_trajectory(sstart, sgoal, T);

  dstart = { sdcar.d, sdcar.d_dot, sdcar.d_dotdot };
  dgoal = { goal_d, 0, 0 };
  d_coeffs = sdcar.jerk_min_trajectory(dstart, dgoal, T);
  printf("  dt:%.2f, Start s, sdot, sdotdot:%.2f, %.2f, %.2f, d: %.2f \n", sdcar.sim_delay, sdcar.s, sdcar.s_dot, sdcar.s_dotdot, sdcar.d);
  printf("   T:%.2f,  Goal s, sdot, sdotdot:%.2f, %.2f, %.2f, d: %.2f \n", T, goal_s, goal_sdot, goal_sdotdot, goal_d);
  printf("s_coeffs: ");
  for(int i =0; i< s_coeffs.size();i++)
  {
    printf("%d: %f ", i, s_coeffs[i]);
  }
  printf("\n");
  int steps=max(sdcar.sim_delay, T) / SIM_dt;

  realize_behavior(sdcar, s_coeffs, d_coeffs, max(steps,100));
}

void BehaviorFSM::realize_behavior(SDVehicle& sdcar, const vector<double>& s_coeff, const vector<double>& d_coeff, int steps)
{
  vector<double> XY, sp_XY, waypointsX, waypointsY;

  // cout << "s coeffs: ";
  //  for(int i = 0; i < s_coeff.size(); i++) {
  //     cout << s_coeff[i] << " ";
  //  }
  // cout << endl;
  double t = 0.;
  double t_max = SIM_dt * steps;
  while(t < t_max) {

    double s_t = s_coeff[0] + s_coeff[1] * t + s_coeff[2] * t * t + s_coeff[3] * t * t * t +
        s_coeff[4] * t * t * t * t + s_coeff[5] * t * t * t * t * t;
    if(!d_coeff.empty()) {
      double d_t = d_coeff[0] + d_coeff[1] * t + d_coeff[2] * t * t + d_coeff[3] * t * t * t +
          d_coeff[4] * t * t * t * t + d_coeff[5] * t * t * t * t * t;
      XY = sdcar.getXY(s_t, d_t);
    } else {
      XY = sdcar.getXY(s_t, sdcar.d);
    }
//    waypointsX.push_back(XY[0]);
//    waypointsY.push_back(XY[1]);
    sdcar.next_x_vals.push_back(XY[0]);
    sdcar.next_y_vals.push_back(XY[1]);
#if 0
    cout << "t,s: (" << t << ", " << s_t << ") ";
    cout << " x,y: (" << XY[0] << ", " << XY[1] << ")" << endl;
#endif

    t += SIM_dt;
  }
  //sort waypoint XY before pass to spline
//  sort_coords(waypointsX, waypointsY);
//
//  // create a spline
//  sp.set_points(waypointsX, waypointsY);
//  for(int i=0; i < waypointsX.size(); i++) {
//    double x = waypointsX[i];
//    sdcar.next_x_vals.push_back(x);
//    sdcar.next_y_vals.push_back(sp(x));
//  }
}

void BehaviorFSM::drive_cs(SDVehicle& sdcar, double vel)
{
  printf("CONSTANT SPEED..\n");
  double dist_inc = vel * SIM_dt;
  for(int i = 0; i < STEPS; i++)
  {
        sdcar.next_x_vals.push_back(sdcar.x+(dist_inc*i)*cos(deg2rad(sdcar.yaw)));
        sdcar.next_y_vals.push_back(sdcar.y+(dist_inc*i)*sin(deg2rad(sdcar.yaw)));
  }
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
    vector<double> s_coeffs, d_coeffs;
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, 3);
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
      generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, 3);

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

void KeepLane::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
  int currlane = get_lane(sdcar.d);
  double acc= MAX_ACC;
  vector<double> s_coeffs, d_coeffs, start_s, goal_s;
  map<int, double> costs;             //(lane, cost)
  vector<double> mincost = { -1, 9 }; //(lane, cost)
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
    cout << "Cost lane 0: " << costs[0] << " lane 1: " << costs[1] << " lane 2: " << costs[2] << endl;

    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }

    // 3. generate straight trajectory
    // there is no trajectory different if the cost of lane 1 is the minimum

    vector<double> s_coeffs, d_coeffs;
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, JMT_T);

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


      if(sdcar.v_ms > 1.0)
      {
        generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, 3);
      }else{
        generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, 2);
      }
      


    // 4. set new state if the cost says to move
//    switch(int(mincost[0])) {
//    case 0:
//
//      set_behavior_state(sdcar, new PrepareLCL("PrepareLCL", 1));
//      break;
//    case 2:
//      set_behavior_state(sdcar, new PrepareLCR("PrepareLCR", 1));
//      break;
//    default:
//#ifdef DEBUG
//      cout << "Keep lane..." << endl;
//#endif
//    }
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
        cout << "Cost lane 0: " << costs[0] << " lane 1: " << costs[1] << " lane 2: " << costs[2] << endl;

    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }

    // 3. generate straight trajectory
    // there is no trajectory different if the cost of lane 1 is the minimum
    vector<double> s_coeffs, d_coeffs;
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, JMT_T);

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
  switch(this->goallane_) {

  case 0: {                       // goal lane 0
    if(fabs(sdcar.d - 2) > 0.2) { // goal not yet reached, generate trajectory
      // 3. generate  trajectoryto lane 0
      printf("Goal d not yet reach \n");
      generate_trajectory(sdcar, this->suggest_acc_, 2, s_coeffs, d_coeffs, 2);

    } else { // goal reach keep lane
      printf("Goal d reached \n");
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
  } break;
  case 1: {                       // goal lane 1
    if(fabs(sdcar.d - 6) > 0.2) { // goal not yet reached, generate trajectory
      // 3. generate  trajectory to lane 1
      generate_trajectory(sdcar, this->suggest_acc_, 6, s_coeffs, d_coeffs, 2);


    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
  } break;
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
  double acc = MAX_ACC;
  vector<double> s_coeffs, d_coeffs, start_s, goal_s;
  vector<deque<Vehicle> > lane_trajectories_0, lane_trajectories_1, lane_trajectories_2;
  map<int, double> costs;             //(lane, cost)
  vector<double> mincost = { -1, 9 }; //(lane, cost)
  switch(currlane) {
  case 1: { // prepare 1 -> 0

    for(const auto& iter_veh : cars_trajectories) {
      if(get_lane(iter_veh.second.back().d) == currlane) {
        lane_trajectories_1.push_back(iter_veh.second);
      } else if(get_lane(iter_veh.second.back().d) == currlane - 1) {
        lane_trajectories_0.push_back(iter_veh.second);
      }
    }
    // 2. calculate the cost for each possible lane
    costs[0] = calc_behaviorlane_cost(sdcar, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, lane_trajectories_1);
    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }

    printf("Cost 0: %f, cost 1: %f", costs[0], costs[1]);
    // whatever the mincost is the robot still drive forward with less vel and acc
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, JMT_T);

    if(mincost[0] == 0) {
#ifdef DEBUG
      cout << "Min Cost is on lane 0 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new LCL("LCL", currlane - 1));
    } else if(mincost[0] == 1) {
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
    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }
    // whatever the mincost is the robot still drive forward, but with slower velocity and acc
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs,JMT_T);

    if(mincost[0] == 1) {
#ifdef DEBUG
      cout << "Min Cost is on lane 1 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new LCL("LCL", currlane - 1));
    } else if(mincost[0] == 2) {
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

  switch(this->goallane_) {
  case 1: {
    if(fabs(sdcar.d - 6) > 0.2) { // goal not yet reached, generate trajectory
      generate_trajectory(sdcar, this->suggest_acc_, 6, s_coeffs, d_coeffs, 2);
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
  } break;
  case 2: {
    if(fabs(sdcar.d - 10) > 0.2) { // goal not yet reached, generate trajectory
      generate_trajectory(sdcar, this->suggest_acc_, 10, s_coeffs, d_coeffs, 2);
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
  } break;
  }
}
PrepareLCR::~PrepareLCR()
{
}

void PrepareLCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  int currlane = get_lane(sdcar.d);
  double acc = MAX_ACC;
  vector<deque<Vehicle> > lane_trajectories_0, lane_trajectories_1, lane_trajectories_2;
  vector<double> s_coeffs, d_coeffs, start_s, goal_s, start_d, goal_d;
  map<int, double> costs;             //(lane, cost)
  vector<double> mincost = { -1, 9 }; //(lane, cost)
  switch(currlane) {
  // Robot on lane 0 praparing to lane 1
  case 0: {
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
    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }

    // whatever the mincost is the robot still drive forward
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d ,s_coeffs, d_coeffs, JMT_T);

    if(mincost[0] == 0) {
#ifdef DEBUG
      cout << "Min Cost is on lane 0 "
           << " val: " << costs[0] << endl;
#endif
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));

    } else if(mincost[0] == 1) {
      set_behavior_state(sdcar, new LCR("LCR", currlane + 1));
    }

  } break;
  //--------------
  // Robot on lane 1->2
  //--------------
  case 1: {

    //(lane, cost)

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
    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mincost[1]) {
        mincost[0] = itercost.first;
        mincost[1] = itercost.second;
      }
    }

    // whatever the mincost is the robot still drive forward, but with slower velocity and acc
    generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, JMT_T);


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
