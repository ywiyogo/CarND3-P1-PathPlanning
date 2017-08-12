#include "behaviorfsm.h"
#include "helper.h"
#include "spline/spline.h"
#include <iostream>

#define DEBUG 0

using namespace std;
using namespace Helper;

const int horizon_t = 1;
const int PRED_TIME = 1;

const int CL_MAX_VEL = 12;  // m/s
const int MAX_JERK = 10;    // in m/s3
const int MAX_ACC = 9;      // in m/s2
const int DIST_BUFFER = 35; // in m
const double V_BUFFER = 5;
const double SIM_dt = 0.02; // 0.02s
const double JMT_T = 2;     // in s for JMT
const int STEPS = 50;
double max_dist = MAX_VEL * PRED_TIME;
const double NORMAL_dV = 0.12;
const double CAUTIOUS_dV = -0.1;
const double ALERT_dV = -0.15;
// Cost value priority weight
const int COLL_W = 4;
const int DIST_W = 3;
const int LANE_W = 1;
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
  this->drive_mode_ = 0;
  cout << "\n##########################################################" << endl;
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

/*
 * ego_car(s, d, v)
 * pred_vehicles[id, x, y, vx,vy, s,d]
 */
double BehaviorFSM::calc_behaviorlane_cost(SDVehicle& sdcar, int lane, vector<deque<Vehicle> >& inlane_veh_trajectories)
{
  double totalcost = 0.;
  double lane_cost = 0.;
  double time_to_collision_cost = 0.;
  double distance_cost = 0.;
  double dist = 0.0;

  deque<Vehicle> frontcar, rearcar;

  // calculate the distance and time to collision cost
  find_closest_cars_inlane(sdcar.s, inlane_veh_trajectories, frontcar, rearcar);
  int size = frontcar.size();
  int cur_idx;
  if(size > 1) {
    cur_idx = size - 2;
  } else if(size > 0) {
    cur_idx = size - 1;
  } else {
    cur_idx = 0;
  }
  // calculate the lane change cost
  int diff_lane = abs(get_lane(sdcar.d) - lane);

  lane_cost = diff_lane;
  // cout << "Lane " << get_lane(frontcar[size - 1].d) << endl;

  if(frontcar.size() == 0) { // no car in front of the ego
    distance_cost = 0.;
    this->drive_mode_ = NORMAL;
  } else {

    vector<double> egoXY = sdcar.getXY(sdcar.s, sdcar.d);
    vector<double> frontXY = sdcar.getXY(frontcar[cur_idx].s, frontcar[cur_idx].d);
    dist = calc_distance(egoXY[0], egoXY[1], frontXY[0], frontXY[1]);
    distance_cost = 10 * exp(-dist / 10);

    // check if the lane is the same as the ego's lane
    if(get_lane(sdcar.d) == lane) {
      // printf("Front car in lane detected, dist: %.2f!\n", dist);
      if(dist < (DIST_BUFFER / 2)) {
        this->drive_mode_ = ALERT;
      } else if(dist < DIST_BUFFER) {
        this->drive_mode_ = CAUTIOUS;
      } else {
        this->drive_mode_ = NORMAL;
      }
    } else { // different lane is important in case of change lane
      if(dist < DIST_BUFFER) {
        // cout << "\n Front car other lane " << lane << " id: " << frontcar[cur_idx].id << " distance: " << dist <<
        // endl;
        totalcost += 3;
      } else {
        if(frontcar[size - 1].v_ms > (sdcar.v_ms)) {
          // cout << "Front car " << frontcar[size - 1].id << " speed is higher than self" << endl;
          totalcost += 0;
        } else {
          totalcost += 0.2;
        }
      }
    }
  }

  // calculate time to collision with the rear car of other lane
  size = rearcar.size();
  if(size == 0) {
    time_to_collision_cost = 0;
  } else if(diff_lane != 0) {

    double pred_s_ego = sdcar.s + sdcar.v_ms * 1;                          // predict s in 1 second
    double pred_s_rear = rearcar[size - 1].s + rearcar[size - 1].v_ms * 1; // predict s in 1 second

    dist = fabs(pred_s_ego - pred_s_rear);

    double coll_t = dist / sdcar.v_ms;
    if(coll_t < 3) {
      time_to_collision_cost = 10 * exp(-coll_t/0.4);
    } else {
      time_to_collision_cost = 0;
    }

    printf("Rear cars lane %d, dist: %.2f, collision time: %.2f, time to collision: %.2f\n", lane, dist, coll_t,
        time_to_collision_cost);
  }
  totalcost = time_to_collision_cost * COLL_W + distance_cost * DIST_W + lane_cost * LANE_W;

  return totalcost;
}

MinCost BehaviorFSM::calc_min_cost(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_predictions, int curr_lane)
{
  vector<deque<Vehicle> > lane_trajectories_0, lane_trajectories_1, lane_trajectories_2;
  map<int, double> costs = { { 0, 999 }, { 1, 999 }, { 2, 999 } }; //(lane, cost)
  MinCost mcost = { -1, 999 };
  for(const auto& iter_veh : cars_predictions) {
    if(get_lane(iter_veh.second.back().d) == 0) {
      lane_trajectories_0.push_back(iter_veh.second);
    } else if(get_lane(iter_veh.second.back().d) == 1) {
      lane_trajectories_1.push_back(iter_veh.second);
    } else if(get_lane(iter_veh.second.back().d) == 2) {
      lane_trajectories_2.push_back(iter_veh.second);
    }
  }
  // 1. check the possible lane
  switch(curr_lane) {
  case LANE0:
    costs[0] = calc_behaviorlane_cost(sdcar, 0, lane_trajectories_0);
    costs[1] = calc_behaviorlane_cost(sdcar, 1, lane_trajectories_1);
    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mcost.cost) {
        mcost.lane = itercost.first;
        mcost.cost = itercost.second;
      }
    }
    break;
  case LANE1:
    costs[0] = calc_behaviorlane_cost(sdcar, 0, lane_trajectories_0);

    costs[1] = calc_behaviorlane_cost(sdcar, 1, lane_trajectories_1);

    costs[2] = calc_behaviorlane_cost(sdcar, 2, lane_trajectories_2);

    for(const auto& itercost : costs) {
      if(itercost.second < mcost.cost) {
        mcost.lane = itercost.first;
        mcost.cost = itercost.second;
      }
    }
    break;
  case LANE2:
    costs[1] = calc_behaviorlane_cost(sdcar, 1, lane_trajectories_1);
    costs[2] = calc_behaviorlane_cost(sdcar, 2, lane_trajectories_2);

    // compare the cost
    for(const auto& itercost : costs) {
      if(itercost.second < mcost.cost) {
        mcost.lane = itercost.first;
        mcost.cost = itercost.second;
      }
    }
    break;
  default:
    printf(" Invalid lane: %d", curr_lane);
  }
  cout << "Cost lane 0: " << costs[0] << " lane 1: " << costs[1] << " lane 2: " << costs[2] << endl;
  printf("---------------\nMin Cost is on lane: %d, val: %.2f\n---------------\n", mcost.lane, mcost.cost);

  return mcost;
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
  int front_id = -1, rear_id = -1;

  if(inlane_veh_trajectories.empty()) {
    return;
  }
  int size = inlane_veh_trajectories.size();

  for(int i = 0; i < size; i++) {
    int trajectory_size = inlane_veh_trajectories[i].size();
    double curr_s = inlane_veh_trajectories[i][size - 1].s;
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

  if(front_id > -1) {
    res_frontcar = inlane_veh_trajectories[front_id];
  }
  if(rear_id > -1) {
    res_rearcar = inlane_veh_trajectories[rear_id];
  }

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
  double ds = acc * T * T;
  if(ds > MAX_VEL) {
    ds = MAX_VEL; // max distance is in second ~ MAX_VELOCITY
  }

  goal_s = sdcar.s + ds;

  double goal_sdot = min(acc * sdcar.sim_delay, double(MAX_VEL));
  printf("given acc: %f, acc*T: %f, v: %f\n", acc, acc * sdcar.sim_delay, goal_sdot);
  double goal_sdotdot = min(acc, double(MAX_ACC));

  sgoal = { goal_s, goal_sdot, 0. };
  s_coeffs = sdcar.jerk_min_trajectory(sstart, sgoal, T);

  dstart = { sdcar.d, sdcar.d_dot, sdcar.d_dotdot };
  dgoal = { goal_d, 0, 0 };
  d_coeffs = sdcar.jerk_min_trajectory(dstart, dgoal, T);
  printf("  dt:%.2f, Start s, sdot, sdotdot:%.2f, %.2f, %.2f, d: %.2f \n", sdcar.sim_delay, sdcar.s, sdcar.s_dot,
      sdcar.s_dotdot, sdcar.d);
  printf("   T:%.2f,  Goal s, sdot, sdotdot:%.2f, %.2f, %.2f, d: %.2f \n", T, goal_s, goal_sdot, goal_sdotdot, goal_d);
  printf("s_coeffs: ");
  for(int i = 0; i < s_coeffs.size(); i++) {
    printf("%d: %f ", i, s_coeffs[i]);
  }
  printf("\n");
  int steps = max(sdcar.sim_delay, T) / SIM_dt;

  realize_behavior(sdcar, s_coeffs, d_coeffs, max(steps, 100));
}

void BehaviorFSM::realize_behavior(SDVehicle& sdcar,
    const vector<double>& s_coeff,
    const vector<double>& d_coeff,
    int steps)
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
  // sort waypoint XY before pass to spline
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

//################################################################
// Behaviour State Definitions
//################################################################

Ready::~Ready()
{
}

void Ready::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{

  if(sdcar.v_ms > 0) {
    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms < MAX_VEL) {
        sdcar.adjust_speed(min(0.4, MAX_VEL - sdcar.v_ms));
      }
      break;
    default:
      printf("FSM: Ready, drive mode: %d\n", this->drive_mode_);
    }
    sdcar.drive(sdcar.ref_v_, 2 + 4 * get_lane(sdcar.d));
    set_behavior_state(sdcar, new KeepLane("KeepLane", get_lane(sdcar.d)));

  } else {
    this->drive_mode_ = NORMAL;

    sdcar.drive(2.0, 2 + 4 * get_lane(sdcar.d));
  }
}

//==================================================================

KeepLane::~KeepLane()
{
}

void KeepLane::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
  int currlane = get_lane(sdcar.d);
  //  double dynamic_dv = 2* sdcar.sim_delay;
  //  printf("## dv: %.2f\n", dynamic_dv);

  MinCost mcost;
  mcost = calc_min_cost(sdcar, cars_trajectories, currlane);

  if((fabs(sdcar.sim_delay) < 0.08) && (this->drive_mode_ == NORMAL)) {
    printf("sdcar v: %.2f \n", sdcar.v_ms);
    this->drive_mode_ = NO_ACC;
  }
  if(fabs(sdcar.d_yaw) > 0.02 && sdcar.v_ms > 4 && (this->drive_mode_ == NORMAL)) { //~10 deg
    printf("\n!!!!! sharp curve -> reduce speed!");
    this->drive_mode_ = CAUTIOUS;
  }
  // 1. check the possible lane
  switch(currlane) {
  //------------------------
  // Lane 0
  //------------------------
  case LANE0:
    cout << "Lane 0" << endl;

    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms < MAX_VEL) {
        sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
      }
      break;
    case CAUTIOUS:
      printf("CAUTIOUS!!");
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      sdcar.adjust_speed(ALERT_dV); // in m/s
      break;
    default:
      printf("%d. No speed change!\n", this->drive_mode_);
    }

    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE0);
    // generate_trajectory(sdcar, this->suggest_acc_, sdcar.d, s_coeffs, d_coeffs, JMT_T);

    // 4. set new state if the cost says to move
    switch(mcost.lane) {
    case LANE1:
      set_behavior_state(sdcar, new PrepareLCR("PrepareLCR", LANE1));
      break;
    default:
      cout << "Keep lane..." << endl;
    }
    break;
  //------------------------
  // Lane 1
  //------------------------
  case LANE1:
    printf("Drive mode: ");
    switch(this->drive_mode_) {
    case NORMAL:
      printf("NORMAL!!");
      if(sdcar.v_ms < MAX_VEL) {
        sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
      }
      break;
    case CAUTIOUS:
      printf("CAUTIOUS!!");
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      printf("ALERT!!");
      sdcar.adjust_speed(ALERT_dV); // in m/s
    default:
      printf("%d. No speed change!\n", this->drive_mode_);
    }
    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE1);

    // 4. set new state if the cost says to move
    if(sdcar.v_ms > MIN_VEL) {
      switch(mcost.lane) {
      case 0:
        set_behavior_state(sdcar, new PrepareLCL("PrepareLCL", LANE0));
        break;
      case 2:
        set_behavior_state(sdcar, new PrepareLCR("PrepareLCR", LANE2));
        break;
      default:
        cout << "Keep lane..." << endl;
      }
    }
    break;
  case LANE2:
    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms < MAX_VEL) {
        sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
      }
      break;
    case CAUTIOUS:
      printf("CAUTIOUS!!");
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      sdcar.adjust_speed(ALERT_dV); // in m/s
    default:
      printf("%d. No speed change!\n", this->drive_mode_);
    }
    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE2);

    // 4. set new state if the cost says to move
    switch(mcost.lane) {
    case LANE1:
      set_behavior_state(sdcar, new PrepareLCL("PrepareLCL", LANE1));
      break;
    default:
      cout << "Keep lane..." << endl;
    }
    break;
  }
}

//==========================================================================
// Prepare Lane Change Left
//==========================================================================
PrepareLCL::~PrepareLCL()
{
  //    # only consider states which can be reached from current FSM state.
  //    possible_successor_states = successor_states(current_fsm_state)
}

void PrepareLCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  // Prepare LCL state can only be perform if the robot drives on lane 1 or lane 2
  int currlane = get_lane(sdcar.d);
  MinCost mcost;
  mcost = calc_min_cost(sdcar, cars_trajectories, currlane);
  switch(currlane) {
  case LANE1: // prepare 1 -> 0

    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms > CL_MAX_VEL) {
        sdcar.adjust_speed(CAUTIOUS_dV);
      }
      break;
    case CAUTIOUS:
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      sdcar.adjust_speed(ALERT_dV); // in m/s
    }
    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE1);

    // 4. set new state if the cost says to move
    switch(mcost.lane) {
    case LANE0:
      if(sdcar.v_ms <= CL_MAX_VEL) {
        set_behavior_state(sdcar, new LCL("LCL", LANE0));
      }
      break;
    case LANE1:
      set_behavior_state(sdcar, new KeepLane("KeepLane", LANE1));
      cout << "Keep lane 1..." << endl;

      break;

    default:
      printf("--- Invalid lane try to change to lane 2!!");
    }
    break;
  //--------------
  // Robot on lane 2 -> 1
  //--------------
  case LANE2:
    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms > CL_MAX_VEL) {
        sdcar.adjust_speed(-0.2);
      }
      break;
    case CAUTIOUS:
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      sdcar.adjust_speed(ALERT_dV); // in m/s
    }
    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE2);

    // 4. set new state if the cost says to move
    switch(mcost.lane) {
    case LANE1:
      set_behavior_state(sdcar, new LCL("LCL", LANE1));
      break;
    case LANE2:
      set_behavior_state(sdcar, new KeepLane("KeepLane", LANE2));
      cout << "Keep lane 2..." << endl;
      break;
    default:
      printf("Invalid lane: %d", mcost.lane);
    }
  }
}
//==========================================================================
// Prepare Lane Change Right
//==========================================================================
PrepareLCR::~PrepareLCR()
{
}

void PrepareLCR::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  int currlane = get_lane(sdcar.d);
  MinCost mcost;
  mcost = calc_min_cost(sdcar, cars_trajectories, currlane);
  switch(currlane) {
  // Robot on lane 0 praparing to lane 1
  case LANE0:

    // whatever the mincost is the robot still drive forward
    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms > CL_MAX_VEL) {
        sdcar.adjust_speed(CAUTIOUS_dV);
      }
      break;
    case CAUTIOUS:
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      sdcar.adjust_speed(ALERT_dV); // in m/s
    }

    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE0);

    switch(mcost.lane) {
    case LANE0:
      set_behavior_state(sdcar, new KeepLane("KeepLane", LANE0));
      cout << "Keep lane 0..." << endl;

      break;
    case LANE1:
      set_behavior_state(sdcar, new LCR("LCR", LANE1));
      break;
    default:
      printf("Invalid lane: %d", mcost.lane);
    }

    break;
  //--------------
  // Robot on lane 1->2
  //--------------
  case LANE1:
    // whatever the mincost is the robot still drive forward, but with slower velocity and acc
    switch(this->drive_mode_) {
    case NORMAL:
      if(sdcar.v_ms > CL_MAX_VEL) {
        sdcar.adjust_speed(CAUTIOUS_dV);
      }
      break;
    case CAUTIOUS:
      sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
      break;
    case ALERT:
      sdcar.adjust_speed(ALERT_dV); // in m/s
    }
    sdcar.drive(sdcar.ref_v_, 2 + 4 * LANE1);

    switch(mcost.lane) {
    case LANE1:
      set_behavior_state(sdcar, new KeepLane("KeepLane", LANE1));
      cout << "Keep lane 1..." << endl;

      break;
    case LANE2:
      set_behavior_state(sdcar, new LCR("LCR", LANE2));
      break;
    default:
      printf("Invalid lane: %d", mcost.lane);
    }

    break;

  default:
    cout << "Invalid lane in lane : " << currlane << endl;
  }
}

//==========================================================================
// Lane Change Left
//==========================================================================
LCL::~LCL()
{
}

void LCL::update_env(SDVehicle& sdcar, map<int, deque<Vehicle> > cars_trajectories)
{
  // In LCL state the robot will drive to left. In this case there are 2 possibilities:
  // from lane 1 to 0 and from lane 2 to 1. During the transitions the robot will drive until
  // the d value of the goal lane is reached. If this is reach then the robot can keep on the lane
  int currlane = get_lane(sdcar.d);

  if(sdcar.v_ms > CL_MAX_VEL) {
    this->drive_mode_ = CAUTIOUS;
  } else {
    this->drive_mode_ = NORMAL;
  }
  // 1. check the possible lane
  switch(this->goallane_) {

  case LANE0:                     // goal lane 0
    if(fabs(sdcar.d - 2) > 0.2) { // goal not yet reached, generate trajectory
      // 3. generate  trajectoryto lane 0
      printf("Goal d not yet reach \n");
      switch(this->drive_mode_) {
      case NORMAL:
        if(sdcar.v_ms < MAX_VEL) {
          sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
        }
        break;
      case CAUTIOUS:
        sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
        break;
      case ALERT:
        sdcar.adjust_speed(ALERT_dV); // in m/s
      }
    } else { // goal reach keep lane
      printf("Goal d reached \n");
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
    sdcar.drive(sdcar.ref_v_, 2);
    break;
  case LANE1: // goal lane 1

    if(fabs(sdcar.d - 6) > 0.2) { // goal not yet reached, generate trajectory
      switch(this->drive_mode_) {
      case NORMAL:
        if(sdcar.v_ms < MAX_VEL) {
          sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
        }
        break;
      case CAUTIOUS:
        sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
        break;
      case ALERT:
        sdcar.adjust_speed(ALERT_dV); // in m/s
      }
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
    sdcar.drive(sdcar.ref_v_, 6);
    break;
  default:
    printf("Invalid lane\n");
  }
}

//==========================================================================
// Lane Change Right
//==========================================================================

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
  if(sdcar.v_ms > CL_MAX_VEL) {
    this->drive_mode_ = CAUTIOUS;
  } else {
    this->drive_mode_ = NORMAL;
  }

  switch(this->goallane_) {
  case LANE1: {
    if(fabs(sdcar.d - 6) > 0.2) { // goal not yet reached, generate trajectory
      switch(this->drive_mode_) {
      case NORMAL:
        if(sdcar.v_ms < MAX_VEL) {
          sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
        }
        break;
      case CAUTIOUS:
        sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
        break;
      case ALERT:
        sdcar.adjust_speed(ALERT_dV); // in m/s
      }
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
    sdcar.drive(sdcar.ref_v_, 6);
  } break;
  case LANE2: {
    if(fabs(sdcar.d - 10) > 0.2) { // goal not yet reached, generate trajectory
      switch(this->drive_mode_) {
      case NORMAL:
        if(sdcar.v_ms < MAX_VEL) {
          sdcar.adjust_speed(min(NORMAL_dV, MAX_VEL - sdcar.v_ms));
        }
        break;
      case CAUTIOUS:
        sdcar.adjust_speed(CAUTIOUS_dV); // in m/s
        break;
      case ALERT:
        sdcar.adjust_speed(ALERT_dV); // in m/s
      }
    } else { // goal reach keep lane
      set_behavior_state(sdcar, new KeepLane("KeepLane", currlane));
    }
    sdcar.drive(sdcar.ref_v_, 10);
  } break;
  }
}
