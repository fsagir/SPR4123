#include "IDM_acc.h"

#include <cmath>
double ACC_idm(double veh_headwy, double veh_length, double veh_rel_spd,
	double time__ln_ch, double current_vel, double desired_vel, double jam__distance, double time__headway, double a_, double b_, double & space_ratio)
{
	double space_headway = veh_headwy - veh_length + veh_rel_spd * time__ln_ch;
	double ratio_upstream_ = current_vel  / desired_vel;
	double desired_space_headway_ = jam__distance + current_vel * time__headway + 0.5 * (current_vel) * (veh_rel_spd) / sqrt(a_ * b_);
	space_ratio = desired_space_headway_ / space_headway;
	return a_ * (1 - pow(ratio_upstream_, 4));
}
/*space_headway_left_upstream = vehicle_headwy_left_upstrm - vehicle_length_left_upstrm + vehicle_rel_spd_left_upstrm * time_ln_ch ;
ratio_left_upstream = (current_velocity - vehicle_rel_spd_left_upstrm) /  vehicle_ID_array[vehicle_ID_left_upstrm] ;
desired_space_headway = jam_distance + (current_velocity - vehicle_rel_spd_left_upstrm) * time_headway + 0.5 * (current_velocity - vehicle_rel_spd_left_upstrm) * (-vehicle_rel_spd_left_upstrm) / sqrt(a * b);
space_ratio = desired_space_headway / space_headway_left_upstream;*/

Vehicle::Vehicle(int V_id_LU, int V_id_RU)
{

}
Vehicle::Vehicle()
{

}