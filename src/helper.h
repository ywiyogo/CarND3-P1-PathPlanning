#ifndef HELPER_H
#define HELPER_H
#include "prediction.h"
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
using namespace std;

namespace Helper
{
const int MAX_VEL = 21.2;     // m/s
const int MIN_VEL = 4;      // m/s
const int CL_MAX_VEL = 14;  // m/s
const int MAX_JERK = 10;    // in m/s3
const int MAX_ACC = 9;      // in m/s2
const int DIST_BUFFER = 35; // in m
const double V_BUFFER = 5;

const double NORMAL_dV = 0.1;
const double CAUTIOUS_dV = -0.1;
const double ALERT_dV = -0.12;

double deg2rad(double x);

double rad2deg(double x);
/*
* Get the lane of a vehicle based on its d value
*/
int get_lane(double d);

double calc_distance(double x1, double y1, double x2, double y2);

double bc_speed_cost(double v);

double bc_collision_cost(double s, double d, const vector<vector<int> >& pred_vehicles);

void sort_coords(vector<double>& v1, vector<double>& v2);
};

#endif // HELPER_H
