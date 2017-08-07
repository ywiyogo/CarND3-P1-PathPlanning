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
/*
* Get the lane of a vehicle based on its d value
*/
int get_lane(double d);

double bc_speed_cost(double v);

double bc_collision_cost(double s, double d, const vector<vector<int> >& pred_vehicles);

void sort_coords(vector<double>& v1, vector<double>& v2);
};

#endif // HELPER_H
