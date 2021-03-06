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

/*
 * 0<d<4 is lane 0
   4<d<8 is lane 1
   8<d<12 is lane 2
 */
int Helper::get_lane(double d)
{
  if(d < 4) {
    return 0;
  } else if(d < 8) {
    return 1;
  } else {
    return 2;
  }
}

double Helper::calc_distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
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

void Helper::sort_coords(vector<double>& v1, vector<double>& v2)
{
  vector<vector<double>> vv;
  int vsize = v1.size();
  for(int i = 0; i < vsize; ++i) {
    vector<double> vt = { v1[i], v2[i] };
    vv.push_back(vt);
  }

  sort(vv.begin(), vv.end());

  v1.clear();
  v2.clear();
  for(int i = 0; i < vsize; ++i) {
    if(i > 0 && vv[i][0] == vv[i - 1][0]) {
      continue;
    }
    v1.push_back(vv[i][0]);
    v2.push_back(vv[i][1]);
  }
}
