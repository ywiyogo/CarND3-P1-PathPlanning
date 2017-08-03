#include "helper.h"
#include <iostream>
#include <math.h>


double Helper::deg2rad(double x)
{
  return x * M_PI / 180;
}
double Helper::rad2deg(double x)
{
  return x * 180 / M_PI;
}

double Helper::bc_speed_cost(double v)
{
  // see lesson 4.11
  double v_limit = 22; // m/s  -> 79.2 kmh
  double buffer_v = 3; // m/s -> 10,8 kmh
  double target_v = v_limit - buffer_v;
  double stop_cost = 0.8;
  double cost = 0.;

  if(v < target_v) {
    cost = stop_cost * ((target_v - v) / target_v);
  } else if(v > target_v && v < v_limit) {
    cost = (v - target_v) / buffer_v;
  } else { // v > target_v
    cost = 1;
  }

  return cost;
}


double Helper::bc_collision_cost(double s, double d, const vector<vector<int> >& pred_vehicles)
{
  double cost = 0.;
  double dist_x, dist_y, distance;
  for(int i = 0; i < pred_vehicles.size(); i++) {
    dist_x = fabs(pred_vehicles[i][5] - s);
    dist_y = fabs(pred_vehicles[i][6] - d);
    distance = dist_x + dist_y;
    if(distance < 2) {
      cost += 1.;
    } else if(distance < 10) {
      // linear equation slope = -1/8, through (2,1)
      cost += -1 / 8. * distance + 1.25;
    } else {
      cost += 0.0;
    }
  }
  return cost;
}

