#ifndef PREDICTION_H
#define PREDICTION_H
#include <vector>
#include <map>
#include <deque>

#include "vehicle.h"

using namespace std;


const unsigned int max_trajectory =5;
class Prediction
{
public:
  Prediction();
  ~Prediction();

void print_curr_trajectories();
void print_trajectories(const map<int, deque<Vehicle> > &prediction_traj);
void update_trajectories(const vector<vector<int>> &sensorfusion);

map<int, deque<Vehicle>> do_prediction(int dt_s = 1);
unsigned int num_vehicles_;

// Map of the car id and its trajectories
map<int,deque<Vehicle>> trajectories_;

};

#endif // PREDICTION_H
