#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU" // for Eigen inverse()
#include "behaviorfsm.h"
#include "helper.h"
#include "sdvehicle.h"
#include "spline/spline.h"
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <string>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace Helper;

const double MAX_S_MAP = 6945.554;
/**
 * Initializes SDVehicle
 */
SDVehicle::SDVehicle()
    : Vehicle()
    , s_dot(0)
    , d_dot(-1)
    , s_dotdot(-1)
    , d_dotdot(-1)
    , prev_v(0)
    , jerk(0.)
    , d_yaw(0.)
    , behaviorfsm_(new Ready("Ready", -1))
    , map_wp_x(0.)
    , map_wp_y(0.)
    , map_wp_s(0.)
    , fdistance(90)
    , sim_delay(0.02)
    , ref_v_(0.)
{
}

SDVehicle::~SDVehicle()
{
  delete behaviorfsm_;
}

/*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
vector<double> SDVehicle::jerk_min_trajectory(vector<double> start, vector<double> end, double T)
{
  MatrixXd A = MatrixXd(3, 3);
  A << T * T * T, T * T * T * T, T * T * T * T * T, 3 * T * T, 4 * T * T * T, 5 * T * T * T * T, 6 * T, 12 * T * T,
      20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T), end[1] - (start[1] + start[2] * T),
      end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = { start[0], start[1], .5 * start[2] };
  for(int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result;
}

void SDVehicle::update_ego(Vehicle& ego,
    const vector<double>& prev_path_x,
    const vector<double>& prev_path_y,
    double dt)
{
  // update car trajectory
  this->prev_path_size = prev_path_x.size();
  this->x = ego.x;
  this->y = ego.y;
  this->global_traj_x_.clear();
  this->global_traj_y_.clear();

  if(prev_path_size < 2) {
    double prev_ref_x = ego.x - cos(ego.yaw);
    double prev_ref_y = ego.y - sin(ego.yaw);

    this->global_traj_x_.push_back(prev_ref_x);
    this->global_traj_x_.push_back(ego.x);

    this->global_traj_y_.push_back(prev_ref_y);
    this->global_traj_y_.push_back(ego.y);

  } else {
    this->x = prev_path_x[prev_path_size - 1];
    this->y = prev_path_y[prev_path_size - 1];
    double prev_ref_x = prev_path_x[prev_path_size - 2];
    double prev_ref_y = prev_path_y[prev_path_size - 2];
    ego.yaw = atan2(this->y - prev_ref_y, this->x - prev_ref_x);

    this->global_traj_x_.push_back(prev_ref_x);
    this->global_traj_x_.push_back(this->x);

    this->global_traj_y_.push_back(prev_ref_y);
    this->global_traj_y_.push_back(this->y);
  }

  this->next_x_vals.clear();
  this->next_y_vals.clear();
  for(int i = 0; i < prev_path_x.size(); i++) {
    this->next_x_vals.push_back(prev_path_x[i]);
    this->next_y_vals.push_back(prev_path_y[i]);
  }

  // updated car states

  this->sim_delay = dt;
  double ds = ego.s - this->s;
  double dd = ego.d - this->d;
  this->d_yaw = ego.yaw - this->yaw;
  if(this->s == -1) {
    this->s_dot = 0;
    this->s_dotdot = 0.;
  } else {
    double cur_s_dot = (ego.s - this->s) / dt;
    this->s_dotdot = (cur_s_dot - this->s_dot) / dt;
    this->s_dot = cur_s_dot;
  }
  if(this->d == -1) {
    this->d_dot = 0;
    this->d_dotdot = 0;
  } else {
    double cur_d_dot = (ego.d - this->d) / dt;
    this->d_dotdot = (cur_d_dot - this->d_dot) / dt;
    this->d_dot = cur_d_dot;
  }

  this->v_ms = ego.v_ms;
  this->s = ego.s;
  this->d = ego.d;
  this->a = (this->v_ms - this->prev_v) / dt;
  this->prev_v = this->v_ms;
  this->yaw = ego.yaw;
  this->acc_list.push_back(this->a);
  double jerk = 0;
  if(this->sim_delay > 0.) {
    int interval = 10; //(int)(1. / this->sim_delay);
    if(this->acc_list.size() >= interval) {
      for(int i = 0; i < this->acc_list.size(); i++) {
        jerk += this->acc_list[i];
      }
      jerk = jerk / this->acc_list.size();
      this->jerk = jerk;
      this->acc_list.pop_front();
    } else {
      printf("Interval: %d, Acc size: %d\n", interval, (int)(this->acc_list.size()));
    }
  }
  cout << "----------------------------------------------------------" << endl;
  cout << "dt: " << this->sim_delay << "x: " << this->x << " y: " << this->y << " yaw: " << this->yaw
       << " dyaw: " << d_yaw << " speed: " << this->v_ms << " a: " << this->a << " jerk: " << this->jerk << endl;
  cout << " s: " << this->s << " ds: " << ds << " s_dot: " << this->s_dot << " s_dotdot: " << this->s_dotdot << endl;
  cout << " d: " << this->d << " dd: " << dd << " d_dot: " << this->d_dot << " d_dotdot: " << this->d_dotdot << endl;
  cout << "----------------------------------------------------------" << endl;
}

// void SDVehicle::update_state(map<int, vector<vector<int> > > predictions)
void SDVehicle::update_env(const map<int, deque<Vehicle> >& vehicle_trajectories, double dt)
{
  this->sim_delay = dt;
  if(this->prev_path_size < 6) {
    behaviorfsm_->update_env(*this, vehicle_trajectories);
  } else {
    printf("### skip update ###\n");
    drive(ref_v_, 2 + 4 * get_lane(d));
  }
  if(dt > .5) {
    printf("WARNING, delay is %f s", dt);
  }
}
void SDVehicle::set_map_waypoints_x(const vector<double>& mwaypoints_x)
{
  this->map_wp_x = mwaypoints_x;
}
void SDVehicle::set_map_waypoints_y(const vector<double>& mwaypoints_y)
{
  this->map_wp_y = mwaypoints_y;
}
void SDVehicle::set_map_waypoints_s(const vector<double>& mwaypoints_s)
{
  this->map_wp_s = mwaypoints_s;
}
void SDVehicle::set_map_waypoints_dx(const vector<double>& mwaypoints_dx)
{
  this->map_wp_dx = mwaypoints_dx;
}
void SDVehicle::set_map_waypoints_dy(const vector<double>& mwaypoints_dy)
{
  this->map_wp_dy = mwaypoints_dy;
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> SDVehicle::getXY(double s, double d)
{
  //  int prev_wp = -1;
  //
  //  while(s > map_wp_s[prev_wp + 1] && (prev_wp < (int)(map_wp_s.size() - 1))) {
  //    prev_wp++;
  //  }
  //
  //  int wp2 = (prev_wp + 1) % map_wp_x.size();
  //
  //  double heading = atan2((map_wp_y[wp2] - map_wp_y[prev_wp]), (map_wp_x[wp2] - map_wp_x[prev_wp]));
  //  // the x,y,s along the segment
  //  double seg_s = (s - map_wp_s[prev_wp]);
  //
  //  double seg_x = map_wp_x[prev_wp] + seg_s * cos(heading);
  //  double seg_y = map_wp_y[prev_wp] + seg_s * sin(heading);
  //
  //  double perp_heading = heading - M_PI / 2;
  //
  //  double x = seg_x + d * cos(perp_heading);
  //  double y = seg_y + d * sin(perp_heading);
  //
  //  return { x, y };

  // implementing the new getXY since the provided function doesn't work well

  // find the index of the s in the map(0 to 6945.554)
  vector<double> s_ranges, x_ranges, y_ranges;
  size_t m_size = map_wp_s.size();
  s = fmod(MAX_S_MAP + s, MAX_S_MAP);

  const vector<double>::iterator& upper_iter = std::upper_bound(map_wp_s.begin(), map_wp_s.end(), s);
  int map_s_index = upper_iter - map_wp_s.begin();
  int prev_wp = map_s_index - 1;

  // get the range between +-3 from the
  for(int i = -3; i < 5; i++) {

    size_t wp = (prev_wp + i + m_size) % m_size;
    double wp_s = map_wp_s[wp];

    x_ranges.push_back(map_wp_x[wp] + d * map_wp_dx[wp]);
    y_ranges.push_back(map_wp_y[wp] + d * map_wp_dy[wp]);

    // Dealing with the circle circuit
    if(prev_wp + i < 0) {
      wp_s -= MAX_S_MAP;
    } else if(prev_wp + i >= m_size) {
      wp_s += MAX_S_MAP;
    }
    s_ranges.push_back(wp_s);
  }

  // Integrate spline to get the way curve by fitting s to x & y
  tk::spline spline_x, spline_y;
  double x, y;
  spline_x.set_points(s_ranges, x_ranges);
  spline_y.set_points(s_ranges, y_ranges);

  x = spline_x(s);
  y = spline_y(s);

  return { x, y };
}

void SDVehicle::drive(double goal_v, double goal_d)
{

  //  double dist_inc = 0.4;
  //  for(int i =0; i<50; i++)
  //  {
  //    double next_s = this->s + (i+1)*dist_inc;
  //    double next_d = 6;
  //    vector<double> XY = this->getXY(next_s, next_d);
  //    this->next_x_vals.push_back(XY[0]);
  //    this->next_y_vals.push_back(XY[1]);
  //
  //  }
  // frondt distance is determined in cost function calculation (the distance cost)
  printf("Front distance: %.2f\n", fdistance);
  // add the next trajectory points sparsely and fill the points between with spline
  vector<double> next_wp0 = this->getXY(this->s + fdistance / 3, goal_d);
  vector<double> next_wp1 = this->getXY(this->s + fdistance * 2 / 3, goal_d);
  vector<double> next_wp2 = this->getXY(this->s + fdistance, goal_d);

  this->global_traj_x_.push_back(next_wp0[0]);
  this->global_traj_x_.push_back(next_wp1[0]);
  this->global_traj_x_.push_back(next_wp2[0]);

  this->global_traj_y_.push_back(next_wp0[1]);
  this->global_traj_y_.push_back(next_wp1[1]);
  this->global_traj_y_.push_back(next_wp2[1]);
  int size = this->global_traj_x_.size();

  printf("Drive to %.2f m/s, d: %2.f \n", goal_v, goal_d);

  //    printf("Global traj x: ");
  //    for(int i = 0; i< size; i++)
  //    {
  //      printf("%.2f ",this->global_traj_x_[i]);
  //    }
  //    printf("\n");
  // Transformation the points to the local car coordinate (shift & rotation)
  // Alternative use Matrix calculation like in my MPC project

  vector<double> local_traj_x(size), local_traj_y(size);
  // transform trajectory points to the car coordinate
  for(int i = 0; i < this->global_traj_x_.size(); i++) {
    double shift_x = this->global_traj_x_[i] - this->x;
    double shift_y = this->global_traj_y_[i] - this->y;

    local_traj_x[i] = (shift_x * cos(0 - this->yaw) - shift_y * sin(0 - this->yaw));
    local_traj_y[i] = (shift_x * sin(0 - this->yaw) + shift_y * cos(0 - this->yaw));
  }
  // printf("\n");

  printf("Local traj x: ");
  for(int i = 0; i < size; i++) {
    printf("%.2f ", local_traj_x[i]);
  }
  printf("\n");

  //  if(local_traj_x[2]< 0.)
  //  {
  //    printf("## ERROR value of next wp! %.2f", local_traj_x[2]);
  //    for(int i = 0; i < this->global_traj_x_.size(); i++) {
  //    double shift_x = this->global_traj_x_[i] - this->x;
  //    double shift_y = this->global_traj_y_[i] - this->y;
  //
  //    local_traj_x[i] = (shift_x * cos(-0.5) - shift_y * sin(-0.5));
  //    local_traj_y[i] = (shift_x * sin(-0.5) + shift_y * cos(-0.5));
  //  }

  tk::spline sp;
  // sort_coords(local_traj_x, local_traj_y);
  sp.set_points(local_traj_x, local_traj_y);

  // Filling the space between the waypoint in order to get the desire velocity
  double target_x = local_traj_x[2];
  double target_y = sp(target_x);
  double target_dist = sqrt(pow(target_x, 2) + pow(target_y, 2));

  double start_x = 0;

  int num_pts = 100;

  //  printf("Traj x: ");
  for(int i = 1; i <= num_pts; i++) {
    double N = target_dist / (0.02 * goal_v);
    double x = start_x + (target_x / N);
    double y = sp(x);
    start_x = x;

    double x_ref = x;
    double y_ref = y;
    // transform back to the global CS
    // rotation
    x = (x_ref * cos(this->yaw) - y_ref * sin(this->yaw));
    y = (x_ref * sin(this->yaw) + y_ref * cos(this->yaw));
    // shift back to global CS
    x += this->x;
    y += this->y;
    //     printf("%.2f, %.2f ",x, y);
    this->next_x_vals.push_back(x);
    this->next_y_vals.push_back(y);
  }
  //   printf("\n");
}

void SDVehicle::adjust_speed(double dv)
{
  printf("\nadjust speed: %.2f \n", dv);
  this->ref_v_ = this->ref_v_ + dv;

  if(this->ref_v_ > MAX_VEL) {
    this->ref_v_ = MAX_VEL;
  }
}

string SDVehicle::get_log()
{
  char log[NAME_MAX];
  snprintf(log, sizeof(log), "%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%s\n", sim_delay, x, y, s, d,
      d_dot, d_dotdot, yaw, d_yaw, v_ms, a, jerk, behaviorfsm_->get_log().c_str());

  return string(log);
}