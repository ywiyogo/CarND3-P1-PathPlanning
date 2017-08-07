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
/*
* Print trajectories for debugging
*/
void print_trajectories(const map<int, deque<Vehicle> > &prediction_traj);
/*
* Update trajectories based on the incoming sensor fusion data
*/
void update_trajectories(const vector<vector<int>> &sensorfusion);

/*
* Perform the trajectory prediction for the next step
* return: a map of all detected vehicles including the predicted states
*/
map<int, deque<Vehicle>> do_prediction(double dt_s = 1.);

// Total number of vehicles
unsigned int num_vehicles_;
// Map of the car id and its trajectories
map<int,deque<Vehicle>> trajectories_;

};

#endif // PREDICTION_H
