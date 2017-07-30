#ifndef COSTFUNCTIONS_H
#define COSTFUNCTIONS_H
#include <vector>
using namespace std;
namespace CostFunctions
{

  double distance_from_goal();
  double change_lane_cost();
  double inefficiency_cost();
  double collision_cost();
  double buffer_cost();
  double calculate_cost(const vector<double> &ego_car, const vector<vector<int>> &pred_vehicles);
};

#endif // COSTFUNCTIONS_H
