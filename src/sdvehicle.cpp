#include "helper.h"
#include "sdvehicle.h"
#include <iostream>
#include <iterator>
#include <map>
#include <math.h>
#include <string>
#include "behaviorfsm.h"
#include "spline/spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/LU" // for Eigen inverse()

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
    , jerk(0)
    , behaviorfsm_(new Ready("Ready",-1))
    , map_wp_x(0.)
    , map_wp_y(0.)
    , map_wp_s(0.)
    , sim_delay(0.02)
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
  A << T * T * T, T * T * T * T, T * T * T * T * T,
       3 * T * T, 4 * T * T * T, 5 * T * T * T * T,
       6 * T, 12 * T * T, 20 * T * T * T;

  MatrixXd B = MatrixXd(3, 1);
  B << end[0] - (start[0] + start[1] * T + .5 * start[2] * T * T),
       end[1] - (start[1] + start[2] * T),
       end[2] - start[2];

  MatrixXd Ai = A.inverse();

  MatrixXd C = Ai * B;

  vector<double> result = { start[0], start[1], .5 * start[2] };
  for(int i = 0; i < C.size(); i++) {
    result.push_back(C.data()[i]);
  }

  return result;
}

void SDVehicle::calc_best_trajectory(vector<double> start_s, vector<double> start_d, int target_veh_id, vector<double> delta, int T, vector<vector<double>> predictions)
{

}

void SDVehicle::realize_state(const vector<double>& predicted_state)
{

}
void SDVehicle::update_ego(EgoVehicle &ego, const vector<double> &prev_path_x, const vector<double> &prev_path_y)
{
  behaviorfsm_->update_ego(*this, ego, prev_path_x, prev_path_y);

}

// void SDVehicle::update_state(map<int, vector<vector<int> > > predictions)
void SDVehicle::update_env(const map<int,deque<Vehicle>> &vehicle_trajectories, double dt)
{

  behaviorfsm_->update_env(*this, vehicle_trajectories);
  this->sim_delay = dt;

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
  int prev_wp = -1;

  while(s > map_wp_s[prev_wp + 1] && (prev_wp < (int)(map_wp_s.size() - 1))) {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % map_wp_x.size();

  double heading = atan2((map_wp_y[wp2] - map_wp_y[prev_wp]), (map_wp_x[wp2] - map_wp_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s - map_wp_s[prev_wp]);

  double seg_x = map_wp_x[prev_wp] + seg_s * cos(heading);
  double seg_y = map_wp_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - M_PI / 2;

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return { x, y };
  // implementing the new getXY since the provided function doesn't work well
  
  //find the index of the s in the map(0 to 6945.554)
//  vector<double> s_ranges, x_ranges, y_ranges;
//  size_t m_size = map_wp_s.size();
//  s= fmod(MAX_S_MAP_MAP + s, MAX_S_MAP_MAP);
//  
//  const vector<double>::iterator &upper_iter = std::upper_bound(map_wp_s.begin(), map_wp_s.end(), s);
//  int map_s_index = upper_iter - map_wp_s.begin() ;
//  int prev_wp = map_s_index -1;
//  
//  //get the range between +-3 from the
//  for(int i = -3; i < 5; i++) {
//    
//    size_t wp = (prev_wp + i + m_size) % m_size;
//    double wp_s = map_wp_s[wp];
//    
//    x_ranges.push_back(map_wp_x[wp] + d * map_wp_dx[wp]);
//    y_ranges.push_back(map_wp_y[wp] + d * map_wp_dy[wp]);
//    
//    //Dealing with the circle circuit
//    if(prev_wp + i < 0) {
//      wp_s -= MAX_S_MAP_MAP;
//    } else if(prev_wp + i >= m_size) {
//      wp_s += MAX_S_MAP_MAP;
//    }
//    s_ranges.push_back(wp_s);
//  }
//
//  // Integrate spline to get the way curve by fitting s to x & y
//  tk::spline spline_x, spline_y;
//  double x,y;
//  spline_x.set_points(s_ranges, x_ranges);
//  spline_y.set_points(s_ranges, y_ranges);
//
//  x = spline_x(s);
//  y = spline_y(s);
//
//  return {x, y};


}