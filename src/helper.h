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

double deg2rad(double x);
double rad2deg(double x);
int get_lane(double d);
double bc_speed_cost(double v);
double bc_collision_cost(double s, double d, const vector<vector<int> >& pred_vehicles);

};

#endif // HELPER_H
