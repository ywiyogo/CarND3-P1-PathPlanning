#include "vehicle.h"

Vehicle::Vehicle():id(-1),x(0),y(0),s(-1),d(-1),v_ms(0),a(0),yaw(-1)
{
}
Vehicle::Vehicle(int id, double s, double d, double v, double a):
id(id),x(0),y(0),s(s),d(d),v_ms(v),a(a),yaw(0)
{
}
Vehicle::~Vehicle()
{
}

