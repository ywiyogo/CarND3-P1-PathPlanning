#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle
{
public:
  Vehicle();
  Vehicle(int id, double s, double d, double v, double a);
  ~Vehicle();

  int id;
  double x, y;
  double s, d;
  double v_ms;
  double a;
  double yaw; //in radian
  
};

#endif // VEHICLE_H
