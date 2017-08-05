#include "prediction.h"
#include <iostream>
#include <math.h>
#include "helper.h"

using namespace Helper;

Prediction::Prediction()
    : num_vehicles_(0)
{
}

Prediction::~Prediction()
{
}
bool is_registered(int id)
{
}
void Prediction::print_curr_trajectories()
{
  cout<<"Print current trajectories"<<endl;
  for(auto const& iter : trajectories_) {
    cout<<"id: "<<iter.first<<" ";
    for(int i=0; i<iter.second.size();i++)
    {
      cout<<"s: "<<iter.second.at(i).s <<" d: "<<iter.second.at(i).d<<" v: "<<iter.second.at(i).v_ms<<" | ";
    }
    cout<<endl;
  }
}
void Prediction::print_trajectories(const map<int, deque<Vehicle> > &prediction_traj)
{
  cout<<"Print predicted trajectories"<<endl;
  for(auto const& iter : prediction_traj) {
    cout<<"id: "<<iter.first<<" ";
    for(int i=0; i<iter.second.size();i++)
    {
      cout<<"s: "<<iter.second.at(i).s <<" d: "<<iter.second.at(i).d<<" v: "<<iter.second.at(i).v_ms<<" | ";
    }
    cout<<endl;
  }
}

void Prediction::update_trajectories(const vector<vector<int> >& sensorfusion)
{
  if(num_vehicles_ == 0) {
    for(int i = 0; i < sensorfusion.size(); i++) {
      Vehicle car;
      car.id = sensorfusion[i][0];
      car.s = sensorfusion[i][5];
      car.d = sensorfusion[i][6];
      car.v_ms = sqrt( pow(sensorfusion[i][3],2) + pow(sensorfusion[i][4],2) );
      deque<Vehicle> trajectory = { car };
      trajectories_[car.id] = trajectory;
    }
  } else {
    for(int i = 0; i < sensorfusion.size(); i++) {
      Vehicle car;
      car.id = sensorfusion[i][0];
      car.s = sensorfusion[i][5];
      car.d = sensorfusion[i][6];
      car.v_ms = sqrt(pow(sensorfusion[i][3], 2) +
          pow(sensorfusion[i][4], 2));       // car.s - trajectories_[car.id][trajectories_[car.id].size() - 1].s;
      if(trajectories_.find(car.id) == trajectories_.end()) { // id doesn't exist
        cout << " -----------\n A new car id is detected !\n -------------" << endl;
        deque<Vehicle> newtrajectory = { car };
        trajectories_[car.id] = newtrajectory;

      } else {                                               // new car id detected
        if(trajectories_[car.id].size() <= max_trajectory) { // max trajectory not exceeded
          trajectories_[car.id].push_back(car);
        } else { // delete old entry
          trajectories_[car.id].pop_front();
          trajectories_[car.id].push_back(car);
        }
      }
    }
  }
  num_vehicles_ = sensorfusion.size();
}

map<int, deque<Vehicle> > Prediction::do_prediction(int dt_s)
{
  map<int, deque<Vehicle> > pred_trajectories = trajectories_;

  for(auto const& iter : trajectories_) {
    double avg_v = 0;
    for(int i = 0; i < iter.second.size(); i++) {
      avg_v += iter.second[i].v_ms;
    }

    avg_v = avg_v / iter.second.size();
    Vehicle predition_car;
    predition_car.id = iter.first;
    //predition_car.s = iter.second[iter.second.size() - 1].s + avg_v;
    predition_car.s = iter.second.back().s + (iter.second.back().v_ms * dt_s);
    predition_car.d = iter.second.back().d; // assuming yaw = 0
    predition_car.v_ms = iter.second.back().v_ms;
    pred_trajectories[iter.first].push_back(predition_car);
  }
  //print_trajectories(pred_trajectories);
  return pred_trajectories;
}