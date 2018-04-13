#include "IDM_acc.h"

#include <cmath>
#include <Utils.h>
double ACC_idm(double veh_headwy, double veh_length, double veh_rel_spd,
	double time__ln_ch, double current_vel, double desired_vel, double jam__distance, double time__headway, double a_, double b_, double & space_ratio)
{
	double errorAcc = 0;
	double space_headway = veh_headwy - veh_length + veh_rel_spd * time__ln_ch;
	double ratio_upstream_ = current_vel  / desired_vel;
	double desired_space_headway_ = jam__distance + current_vel * time__headway + 0.5 * (current_vel) * (veh_rel_spd) / sqrt(a_ * b_);
	space_ratio = desired_space_headway_ / space_headway;
	double accIDM = a_ * (1 - pow(ratio_upstream_, 4));
#if defined(SAE0_CAR) || defined(SAE0_TRUCK)
	double threshold = 0.01;	// 0.005 - 0.015
#else if defined(SAE1_CAR) || defined(SAE1_TRUCK) || define(SAE2_CAR) || defined(SAE2_TRUCK) || defined(SAE3_CAR) || defined(SAE3_TRUCK) || defined(SAE4_TRUCK) || defined(SAE4_CAR) || defined(SAE5_CAR) || defined(SAE5_TRUCK)
	double threshold = 0.001;	// 0.0005 - 0.0015
#endif
	errorAcc = utils::NormalDistribution::getTruncatedNormalDouble(0, threshold, utils::DistributionTruncationFrom_0_5_To_1_5(threshold));
	//return (accIDM < 0.0)? accIDM : accIDM + errorAcc;
	return accIDM;
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