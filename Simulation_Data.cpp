//include header file describing data
#include "Simulation_Data.h"
#include "IDM_acc.h"
#include <cmath>
#include <random>
#include <chrono>
#include <algorithm>

//Variables declared here are defined in Simulation_Data.h
long		cur_link = -1;
long	   cur_veh_lane = -1;
long	 number_of_lanes = -1;
double  desired_acceleration = 0.0;
double  desired_lane_angle = 0.0;
long    active_lane_change = 0;
long    rel_target_lane = 0;
double  desired_velocity = 0.0;
long    turning_indicator = 0;
long    vehicle_color = RGB(0, 0, 0);
double  current_velocity = 0.0;
double  current_acceleration = 0.0;
double  time_step = 0.0;

std::map<int, long> vehicleLaneChangeMap;

//Default values depend on preprocessor Macro (set in project configuration)
#ifdef SAE0_CAR
double  a = 1.4;
double  time_headway = 1.5;
#elif SAE1_CAR
double  a = 1.4;
double  time_headway = 1.5;
#elif SAE2_CAR
double  a = 1.4;
double  time_headway = 1.5;
#elif SAE3_CAR
double  a = 1.4;
double  time_headway = 1.5;
#elif SAE4_CAR
double  a = 1.4;
double  time_headway = 1.5;
#elif SAE5_CAR
double  a = 1.4;
double  time_headway = 1.5;
#elif SAE0_TRUCK
double  a = 0.7;
double  time_headway = 2.0;
#elif SAE1_TRUCK
double  a = 0.7;
double  time_headway = 2.0;
#elif SAE2_TRUCK
double  a = 0.7;
double  time_headway = 2.0;
#elif SAE3_TRUCK
double  a = 0.7;
double  time_headway = 2.0;
#elif SAE4_TRUCK
double  a = 0.7;
double  time_headway = 2.0;
#elif SAE5_TRUCK
double  a = 1.4;
double  time_headway = 1.5;
#else
#error SAE level not set
#endif // SAE0_CAR

//double  a = 3.5;
//double  time_headway = 2.0;
double  b = 2;
double  jam_distance = 2.0;
double  ratio = 0.0;
double  desired_space_headway = 0.0;
double  relative_velocity = 0.0;
double  space_ratio = 0.0;
double  space_headway = 0.0;
double  relative_distance = 10000.0;
double  vehicle_length = 0.0;
double  vehicle_ID = 0.0;
double  leading_veh_spd = 0.0;
double  leading_veh_acc = 0.0;
double  acc_idm = 0.0;
double  acc_cah = 0.0;
double  acc_acc = 0.0;
double  effective_acc = 0.0;
long    heaviside_step = 0;
double  c = 0.9;
double  b_safe = 4.0;
double  time_ln_ch = 0.0;
int     vehicle_ID_left_upstrm = 0;
int     vehicle_ID_right_upstrm = 0;
int     vehicle_ID_left_downstrm = 0;
int     vehicle_ID_right_downstrm = 0;
int     vehicle_ID_current_downstrm = 0;
int     vehicle_ID_current_upstrm = 0;
int     vehicle_ID_current_two_upstream = 0;
int     vehicle_ID_current_two_downstream = 0;
double  vehicle_rel_spd_left_upstrm = 0.0;
double  vehicle_rel_spd_right_upstrm = 0.0;
double  vehicle_rel_spd_left_downstrm = 0.0;
double  vehicle_rel_spd_right_downstrm = 0.0;
double  vehicle_rel_spd_current_downstrm = 0.0;
double  vehicle_rel_spd_current_upstrm = 0.0;
double  vehicle_headwy_left_upstrm = 0.0;
double  vehicle_headwy_right_upstrm = 0.0;
double  vehicle_headwy_left_downstrm = 0.0;
double  vehicle_headwy_right_downstrm = 0.0;
double  vehicle_headwy_current_upstrm = 0.0;
double  vehicle_headwy_current_downstrm = 0.0;
double  vehicle_length_left_upstrm = 0.0;
double  vehicle_length_left_downstrm = 0.0;
double  vehicle_length_right_upstrm = 0.0;
double  vehicle_length_right_downstrm = 0.0;
double  vehicle_length_right_self = 0.0;
double  space_headway_left_upstream = 0.0;
double  space_headway_left_self = 0.0;
double  space_headway_right_upstream = 0.0;
double  space_headway_right_self = 0.0;
double  space_headway_current_upstream = 0.0;
double  vehicle_acc_left_downstrm = 0.0;
double	vehicle_acc_left_upstrm = 0.0;
double  vehicle_acc_right_downstrm = 0.0;
double	vehicle_acc_right_upstrm = 0.0;
double  vehicle_acc_current_downstrm = 0.0;
double	vehicle_acc_current_upstrm = 0.0;
double  ratio_left_upstream = 0.0;
double  ratio_left_downstream = 0.0;
double  ratio_right_upstream = 0.0;
double  ratio_right_downstream = 0.0;
double  ratio_current_upstream = 0.0;
//Arrays of information used during debugging and data output
std::vector<double>    vehicle_desired_vel_array;
std::vector<double>    vehicle_X_coordinate_array;
long    vehicle_identity = 0;
double  acc_idm_left_upstream = 0.0;
double  acc_idm_right_upstream = 0.0;
double  acc_idm_left_self = 0.0;
double  acc_idm_right_self = 0.0;
double  acc_idm_current_upstream = 0.0;
double  p = 1;
double  acc_thr_human = 1;
double  acc_thr_system = .5;
double acc_thr = 2;
double lane_change_to_left = 0.0;
double lane_change_to_right = 0.0;
double current_time = 0.0;
double lateral_position = 0.0;
double current_veh_angle = 0.0;
long VehicleID = -1;
long vehicle_type = 0;
double x_coordinate;
double y_coordinate;
double reaction_time = 0.0;
double desired_angle = 0.0;
//bool lane_change_in_progress = 0;
//bool completion_of_lane_change = 0;
//double change_of_control_on_angle_time = 0;
double duration_of_vissim_conrol_on_angle_after_lane_change = 2;
long MOBIL_active_lane_change = 0;
long lane_change_for_SAE_level = 0;


void CalculateAccChange(double * double_value)
{
	/* Calculation of IDM acceleration - SAE Level 0 */


	space_headway = relative_distance - vehicle_length;
	if (cur_link != 7) {

	desired_velocity = 11.2 + (desired_velocity - 24.4) / (36.12 -24.4) * (15.6-11.2);
	}
	ratio = current_velocity / desired_velocity;
	desired_space_headway = jam_distance + current_velocity * time_headway + 0.5 * current_velocity * relative_velocity / sqrt(a * b);
	space_ratio = desired_space_headway / space_headway;
	/* checking if the vehicle is the first vehicle*/
	if (vehicle_ID == -1) {
		acc_idm = a * (1 - ratio * ratio * ratio * ratio);
	}
	else {
		acc_idm = a * (1 - ratio * ratio * ratio * ratio - space_ratio * space_ratio);
	}
#if defined(SAE0_CAR) || defined(SAE0_TRUCK)

		*double_value = acc_idm;
#else
		/* Calculation of CAH acceleration - will be used to calculate ACC acceleration */
		if (leading_veh_acc > a) {
			effective_acc = a;
		}
		else {
			effective_acc = leading_veh_acc;
		}
		leading_veh_spd = current_velocity - relative_velocity;

		if (relative_velocity <= 0) {
			heaviside_step = 0;
		}
		else {
			heaviside_step = 1;
		}


		if (vehicle_ID == -1) {
			acc_cah = a * (1 - ratio * ratio * ratio * ratio);
		}
		else if (leading_veh_spd * relative_velocity <= -2 * space_headway * effective_acc) {
			acc_cah = (pow(current_velocity, 2) * effective_acc) / (pow(leading_veh_spd, 2) - 2 * space_headway * effective_acc);
		}
		else {
			acc_cah = effective_acc - (pow(relative_velocity, 2) * heaviside_step) / (2 * space_headway);
		}

		/* Calculation of ACC acceleration - SAE Level 1*/

		if (vehicle_ID == -1) {
			acc_acc = a * (1 - ratio * ratio * ratio * ratio);
		}
		else if (acc_idm >= acc_cah) {
			acc_acc = acc_idm;
		}
		else {

			acc_acc = (1 - c) * acc_idm + c * (acc_cah + b * tanh((acc_idm - acc_cah) / b));

			if (acc_acc < acc_idm) {
				acc_acc = acc_idm;
			}
			/*acc_acc = (1 - c) * acc_idm + c * acc_cah;*/
		}

		//Call to detirmine acceleration 
		DetrimeAccValue(double_value);
#endif
}

//Function to detirmine acceleration value
void DetrimeAccValue(double * double_value)
{
	if (VehicleID < 10 || (VehicleID > 150 && VehicleID < 170))
	{
		StoreSituationData(VehicleID);
	}
	if (DataMap[VehicleID].level_shift == Human_Control)
	{
		if (current_time > DataMap[VehicleID].time_to_shift)
			*double_value = acc_idm;
		else
			*double_value = acc_acc;
	}
	else if (DataMap[VehicleID].level_shift == Automated_Control)
	{
		if (current_time > DataMap[VehicleID].time_to_shift)
			*double_value = acc_acc;
		else
			*double_value = acc_idm;
	}

}

void DetermineLatPosValue(double * double_value)
{

	if (DataMap[VehicleID].level_shift == Human_Control )
	{
		if (current_time > DataMap[VehicleID].time_to_shift)
		{
			if (DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 10 / 80 <= 0)
			{
				*double_value = max(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 10 / 80, -.2);
			}

			else
				*double_value = min(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 10 / 80, .2);
		}			//*double_value = desired_lane_angle;
		else
		{
			if (DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 2 / 80 <= 0)
			{
				*double_value = max(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 2 / 80, -.2);
			}

			else
				*double_value = min(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 2 / 80, .2);
			
		}
	}
	else if (DataMap[VehicleID].level_shift == Automated_Control )
	{
		if (current_time > DataMap[VehicleID].time_to_shift)
		{
			if (DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 2 / 80 <= 0)
			{
				*double_value = max(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 2 / 80, -.2);
			}

			else
				*double_value = min(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 2 / 80, .2);

		}
		else
		{
			if (DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 10 / 80 <= 0)
			{
				*double_value = max(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 10 / 80, -.2);
			}

			else
				*double_value = min(DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation() - lateral_position) * 10 / 80, .2);
		}
	}

}

void DetermineLaneChangeValue(long * long_value)
{
	if (DataMap[VehicleID].level_shift == Human_Control)
	{
		if (current_time > DataMap[VehicleID].time_to_shift) {
			*long_value = active_lane_change;
			DataMap[VehicleID].active_lane_change = active_lane_change;
			lane_change_for_SAE_level = active_lane_change;

		}
		else
			CalculateAutomatedLaneChange(long_value);
	}
	else if (DataMap[VehicleID].level_shift == Automated_Control)
	{
		if (current_time > DataMap[VehicleID].time_to_shift)
			CalculateAutomatedLaneChange(long_value);
		else
		{
			*long_value = active_lane_change;
			DataMap[VehicleID].active_lane_change = active_lane_change;
			lane_change_for_SAE_level = active_lane_change;
		}
	}

}

void CalculateAutomatedLaneChange(long* long_value)
{
	lane_change_to_left = 0.0;
	lane_change_to_right = 0.0;
	*long_value = 0;
	if (abs(lateral_position) > 1.75 || current_velocity < desired_velocity / 2)
	{
		lateral_position = 1.75;
		*long_value = 0;
		DataMap[VehicleID].active_lane_change = 0;
		lane_change_for_SAE_level = 0;
		return;
	}

	//if (DataMap[VehicleID].level_shift == Human_Control)
	//{
	//	if (current_time > DataMap[VehicleID].time_to_shift) {
	//		acc_thr = acc_thr_human;
	//	}
	//	else
	//		acc_thr = acc_thr_system;
	//}
	//else if (DataMap[VehicleID].level_shift == Automated_Control)
	//{
	//	if (current_time > DataMap[VehicleID].time_to_shift)
	//		acc_thr = acc_thr_system;
	//	else
	//	{
	//		acc_thr = acc_thr_human;
	//	}
	//}

	number_of_lanes = DataMap[VehicleID].getLaneCount();

	if (vehicleLaneChangeMap.find(VehicleID) != vehicleLaneChangeMap.end()) {
		if (vehicleLaneChangeMap[VehicleID] != 0) {
			*long_value = vehicleLaneChangeMap[VehicleID];
			DataMap[VehicleID].active_lane_change = *long_value;
			return;
		}
	}

	if (number_of_lanes == 1 || DataMap[vehicle_ID_current_upstrm].Lane_change_in_progress == true
		|| DataMap[vehicle_ID].Lane_change_in_progress == true
		|| DataMap[vehicle_ID_current_two_upstream].Lane_change_in_progress == true
		|| DataMap[vehicle_ID_current_two_downstream].Lane_change_in_progress == true)
	{
		/*if (vehicleLaneChangeMap.find(VehicleID) != vehicleLaneChangeMap.end()) {
		*long_value = vehicleLaneChangeMap[VehicleID];
		DataMap[VehicleID].active_lane_change = *long_value;
		}
		else {*/
		*long_value = 0;
		DataMap[VehicleID].active_lane_change = 0;
		//}
		return;
	}
	/*c) Checking for the changed new acceleration of the vehcile behind self when self moves away*/
	//space_headway_current_upstream = vehicle_headwy_current_downstrm + vehicle_headwy_current_upstrm - vehicle_length - (vehicle_rel_spd_current_downstrm - vehicle_rel_spd_current_upstrm) * time_ln_ch;
	//ratio_current_upstream = (current_velocity - vehicle_rel_spd_current_upstrm) / vehicle_ID_array[vehicle_ID_current_upstrm];
	//desired_space_headway = jam_distance + (current_velocity - vehicle_rel_spd_current_upstrm) * time_headway + 0.5 *  (current_velocity - vehicle_rel_spd_current_upstrm) * (vehicle_rel_spd_current_downstrm - vehicle_rel_spd_current_upstrm) / sqrt(a * b);
	//space_ratio = desired_space_headway / space_headway_left_self;
	/* checking if the vehicle is the first vehicle*/
	//if (vehicle_ID == -1) {
	//	acc_idm_current_upstream = ACC_idm(vehicle_headwy_current_downstrm + vehicle_headwy_current_upstrm, vehicle_length, -(vehicle_rel_spd_current_downstrm - vehicle_rel_spd_current_upstrm), time_ln_ch,
	//		(current_velocity - vehicle_rel_spd_current_upstrm), vehicle_desired_vel_array[vehicle_ID_current_upstrm],
	//		jam_distance, time_headway, a, b, space_ratio);
	//}
	if (vehicle_ID_current_upstrm > 0)
		acc_idm_current_upstream = ACC_idm(vehicle_headwy_current_downstrm - vehicle_headwy_current_upstrm, vehicle_length, DataMap[vehicle_ID].current_velocity - DataMap[vehicle_ID_current_upstrm].current_velocity,
			time_ln_ch, DataMap[vehicle_ID_current_upstrm].current_velocity, vehicle_desired_vel_array[vehicle_ID_current_upstrm],
			jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio) - pow(space_ratio, 2);
	else acc_idm_current_upstream = 0;

	if (cur_veh_lane == number_of_lanes /*&& DataMap[vehicle_ID_left_upstrm].Lane_change_in_progress == false
										&& DataMap[vehicle_ID_left_downstrm].Lane_change_in_progress == false*/)
	{
		//leftmost lane

		/*a) Checking for the changed new acceleration of the upstream vehcile on the left*/
		if (vehicle_ID_left_upstrm > 0)
			acc_idm_left_upstream = ACC_idm(-vehicle_headwy_left_upstrm, vehicle_length_left_upstrm, vehicle_rel_spd_left_upstrm,
				time_ln_ch, DataMap[vehicle_ID_left_upstrm].current_velocity, vehicle_desired_vel_array[vehicle_ID_left_upstrm], jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio)
			- pow(space_ratio, 2);
		else acc_idm_left_upstream = 0;

		/*space_headway_left_upstream = vehicle_headwy_left_upstrm - vehicle_length_left_upstrm + vehicle_rel_spd_left_upstrm * time_ln_ch ;
		ratio_left_upstream = (current_velocity - vehicle_rel_spd_left_upstrm) /  vehicle_ID_array[vehicle_ID_left_upstrm] ;
		desired_space_headway = jam_distance + (current_velocity - vehicle_rel_spd_left_upstrm) * time_headway + 0.5 * (current_velocity - vehicle_rel_spd_left_upstrm) * (-vehicle_rel_spd_left_upstrm) / sqrt(a * b);
		space_ratio = desired_space_headway / space_headway_left_upstream;*/


		//acc_idm_left_upstream = a * (1 - pow(ratio_left_upstream, 4) - pow(space_ratio,2));


		/*b) Checking for the changed new acceleration of the self vehcile when it moves to the left*/

		//space_headway_left_self = vehicle_headwy_left_downstrm - vehicle_length_left_downstrm - vehicle_rel_spd_left_downstrm * time_ln_ch;
		//ratio_left_downstream = current_velocity / desired_velocity;
		//desired_space_headway = jam_distance + current_velocity * time_headway + 0.5 * vehicle_rel_spd_left_downstrm * current_velocity / sqrt(a * b);
		//space_ratio = desired_space_headway / space_headway_left_self;
		/* checking if the vehicle is the first vehicle*/
		if (vehicle_ID_left_downstrm == -1) {
			acc_idm_left_self = ACC_idm(vehicle_headwy_left_downstrm, vehicle_length_left_downstrm, -vehicle_rel_spd_left_downstrm,
				time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio);
		}
		else {
			acc_idm_left_self = ACC_idm(vehicle_headwy_left_downstrm, vehicle_length_left_downstrm, -vehicle_rel_spd_left_downstrm,
				time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio) - pow(space_ratio, 2);
		}

		if (!(acc_idm_left_upstream <= -b_safe || acc_idm_left_self <= -b_safe))
		{
			if (acc_idm_left_self - acc_idm + p * (acc_idm_left_upstream - vehicle_acc_left_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm) >= acc_thr)
			{
				lane_change_to_left = acc_idm_left_self - acc_idm + p * (acc_idm_left_upstream - vehicle_acc_left_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm);
			}
		}
	}
	else
	{
		acc_idm_left_upstream = 0;
		acc_idm_left_self = 0;
		lane_change_to_left = 0;
	}
	if (cur_veh_lane == 1 /*&& DataMap[vehicle_ID_right_upstrm].Lane_change_in_progress == false
						  && DataMap[vehicle_ID_right_downstrm].Lane_change_in_progress == false*/)
	{
		//rightmost lane

		/*d) Checking for the changed new acceleration of the upstream vehcile on theright*/
		//space_headway_right_upstream = vehicle_headwy_right_upstrm - vehicle_length_right_upstrm + vehicle_rel_spd_right_upstrm * time_ln_ch;
		//ratio_right_upstream = (current_velocity - vehicle_rel_spd_right_upstrm) / vehicle_ID_array[vehicle_ID_right_upstrm];
		//desired_space_headway = jam_distance + (current_velocity - vehicle_rel_spd_right_upstrm) * time_headway + 0.5 * (current_velocity - vehicle_rel_spd_right_upstrm) * (-vehicle_rel_spd_right_upstrm) / sqrt(a * b);
		//space_ratio = desired_space_headway / space_headway_right_upstream;

		if (vehicle_ID_right_upstrm > 0)
			acc_idm_right_upstream = ACC_idm(-vehicle_headwy_right_upstrm, vehicle_length_right_upstrm, vehicle_rel_spd_right_upstrm,
				time_ln_ch, (current_velocity - vehicle_rel_spd_right_upstrm), vehicle_desired_vel_array[vehicle_ID_right_upstrm], jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio)
			- pow(space_ratio, 2);
		else acc_idm_right_upstream = 0;


		/*e) Checking for the changed new acceleration of the self vehcile when it moves to the right*/
		//space_headway_right_self = vehicle_headwy_right_downstrm - vehicle_length - vehicle_rel_spd_right_downstrm * time_ln_ch;
		//ratio_right_downstream = current_velocity / desired_velocity;
		//desired_space_headway = jam_distance + current_velocity * time_headway + 0.5 * vehicle_rel_spd_right_downstrm * current_velocity / sqrt(a * b);
		//space_ratio = desired_space_headway / space_headway_right_self;
		/* checking if the vehicle is the first vehicle*/
		if (vehicle_ID_right_downstrm == -1) {
			acc_idm_right_self = ACC_idm(vehicle_headwy_right_downstrm, vehicle_length_right_downstrm, -vehicle_rel_spd_right_downstrm,
				time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio);
		}
		else {
			acc_idm_right_self = ACC_idm(vehicle_headwy_right_downstrm, vehicle_length_right_downstrm, -vehicle_rel_spd_right_downstrm,
				time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, DataMap[VehicleID].getConstA(), DataMap[VehicleID].getConstB(), space_ratio) - pow(space_ratio, 2);
		}

		if (!(acc_idm_right_upstream <= -b_safe || acc_idm_right_self <= -b_safe))
		{
			if (acc_idm_right_self - acc_idm + p * (acc_idm_right_upstream - vehicle_acc_right_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm) >= acc_thr)
			{
				lane_change_to_right = acc_idm_right_self - acc_idm + p * (acc_idm_right_upstream - vehicle_acc_right_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm);

			}
		}
	}
	else
	{
		acc_idm_right_upstream = 0;
		acc_idm_right_self = 0;
		lane_change_to_right = 0;
	}
	/*Check if the vehicle should change lane to the left*/


	if ((lane_change_to_left > lane_change_to_right) && (lane_change_to_left >= acc_thr)) {
		if (DataMap[VehicleID].isCurLaneLeftMost(cur_veh_lane)) {
			*long_value = 0;
			DataMap[VehicleID].active_lane_change = 0;
			lane_change_for_SAE_level = 0;
		}
		else {
			*long_value = 1;
			DataMap[VehicleID].active_lane_change = 1;
			lane_change_for_SAE_level = 1;
		}
	}
	else if ((lane_change_to_right > lane_change_to_left) && (lane_change_to_right >= acc_thr)) {
		if (DataMap[VehicleID].isCurLaneRightMost(cur_veh_lane)) {
			*long_value = 0;
			DataMap[VehicleID].active_lane_change = 0;
			lane_change_for_SAE_level = 0;
		}
		else {
			*long_value = -1;
			DataMap[VehicleID].active_lane_change = -1;
			lane_change_for_SAE_level = -1;
		}
	}
	else if ((lane_change_to_right == lane_change_to_left) && (lane_change_to_right > acc_thr)) {
		if (DataMap[VehicleID].isCurLaneLeftMost(cur_veh_lane)) {
			*long_value = -1;
			DataMap[VehicleID].active_lane_change = -1;
			lane_change_for_SAE_level = -1;
		}
		else {
			*long_value = 1;
			DataMap[VehicleID].active_lane_change = 1;
			lane_change_for_SAE_level = 1;
		}
	}
	else {
		if (vehicleLaneChangeMap.find(VehicleID) != vehicleLaneChangeMap.end()) {
			*long_value = vehicleLaneChangeMap[VehicleID];
			DataMap[VehicleID].active_lane_change = *long_value;
			lane_change_for_SAE_level = *long_value;
		}
		else {
			*long_value = 0;
			DataMap[VehicleID].active_lane_change = 0;
			lane_change_for_SAE_level = 0;
		}
	}
#if defined(SAE0_CAR) || defined(SAE0_TRUCK) || defined(SAE1_CAR) || defined(SAE1_TRUCK) || defined(SAE2_CAR) || defined(SAE2_TRUCK)
	//if (false == utils::NormalDistribution::flipCoinWithProbability(LANE_CHANGE_PROBABILITY))
	//	*long_value = 0;
#endif
	vehicleLaneChangeMap[VehicleID] = *long_value;
	//*long_value = (rand() % 3) - 1;
}/*==========================================================================*/

//Setvalue function is called by VISSIM to provide information to simulation

DRIVERMODEL_API  int  DriverModelSetValue(long   type,
	long   index1,
	long   index2,
	long   long_value,
	double double_value,
	char   *string_value)
{

	//generate seed value from time
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	//seed random generator
	std::default_random_engine generator(seed);
	std::lognormal_distribution<double> reaction_time_distribution(0.83, 0.53);
	/*reaction_time = reaction_time_distribution(generator);*/




	/* Sets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, to <long_value>, <double_value> or            */
	/* <*string_value> (object and value selection depending on <type>).    */
	/* Return value is 1 on success, otherwise 0.                           */


	//Call dataSetValue (This function can be found in DataMap, defined in Vehicle_data.h)
	//	This effectively allows us to pass information from Vissim directly to our data collection
	//	upon completetion simulation code below runs normally.
	DataSetValue(type, index1, index2, long_value, double_value, string_value);


	switch (type) {
	case DRIVER_DATA_PATH:
		//
		return 1;
	case DRIVER_DATA_TIMESTEP:
		time_step = double_value;
		return 1;
	case DRIVER_DATA_TIME:
		current_time = double_value;


		return 1;
	case DRIVER_DATA_VEH_ID:
		VehicleID = long_value;

		vehicle_identity = long_value;
		return 1;
	case DRIVER_DATA_VEH_LANE:
		cur_veh_lane = long_value;
	case DRIVER_DATA_VEH_ODOMETER:
		return 1;
	case DRIVER_DATA_VEH_LANE_ANGLE:
		current_veh_angle = double_value;
		return 1;
	case DRIVER_DATA_VEH_LATERAL_POSITION:
		lateral_position = double_value;
		return 1;
	case DRIVER_DATA_VEH_VELOCITY:
		current_velocity = double_value;


		return 1;
	case DRIVER_DATA_VEH_ACCELERATION:
		current_acceleration = double_value;
		return 1;
	case DRIVER_DATA_VEH_LENGTH:
	case DRIVER_DATA_VEH_WIDTH:
	case DRIVER_DATA_VEH_WEIGHT:
	case DRIVER_DATA_VEH_MAX_ACCELERATION:
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		turning_indicator = long_value;
		return 1;
	case DRIVER_DATA_VEH_CATEGORY:
	case DRIVER_DATA_VEH_PREFERRED_REL_LANE:
	case DRIVER_DATA_VEH_USE_PREFERRED_LANE:
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		desired_velocity = double_value;
		if (vehicle_desired_vel_array.size() < vehicle_identity + 1)
			vehicle_desired_vel_array.resize(vehicle_identity + 1);
		vehicle_desired_vel_array[vehicle_identity] = double_value;
		return 1;
	case DRIVER_DATA_VEH_X_COORDINATE:
		x_coordinate = double_value;
		if (vehicle_X_coordinate_array.size() < vehicle_identity + 1)
			vehicle_X_coordinate_array.resize(vehicle_identity + 1);
		vehicle_X_coordinate_array[vehicle_identity] = double_value;
		//if (double_value > -7200 && double_value < -5000)
			StoreVehicleFrame(DataMap[VehicleID]);
		if (double_value >= 10)
		{
			if (DataMap[VehicleID].Volume_set == false)
			{
				VehicleData::Volume++;
				DataMap[VehicleID].Volume_set = true;
			}
		}

	case DRIVER_DATA_VEH_Y_COORDINATE:
		y_coordinate = double_value;

		return 1;

	case DRIVER_DATA_VEH_TYPE:
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		vehicle_color = long_value;
		return 1;
	case DRIVER_DATA_VEH_CURRENT_LINK:
		cur_link = long_value;
		return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
				  /* Must return 1 if these messages are to be sent from VISSIM!         */
	case DRIVER_DATA_VEH_NEXT_LINKS:
		break;
	case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE:
		if (active_lane_change != 0 && long_value == 0)
			active_lane_change = 0;
		vehicleLaneChangeMap[VehicleID] = long_value;
		break;
	case DRIVER_DATA_VEH_REL_TARGET_LANE:
	case DRIVER_DATA_NVEH_ID:
		/*To identify if any vehicle is in front of the current vehicle*/
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_ID = long_value;
		}
		/*storing vehcile id for lane changing operations*/
		if ((index1 == 1) && (index2 == 1)) {
			vehicle_ID_left_downstrm = long_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			vehicle_ID_left_upstrm = long_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			vehicle_ID_right_downstrm = long_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			vehicle_ID_right_upstrm = long_value;
		}
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_ID_current_downstrm = long_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			vehicle_ID_current_upstrm = long_value;
		}
		if ((index1 == 0) && (index2 == 2)) {
			vehicle_ID_current_two_downstream = long_value;
		}
		if ((index1 == 0) && (index2 == -2)) {
			vehicle_ID_current_two_upstream = long_value;
		}

		return 1;
	case DRIVER_DATA_NVEH_LANE_ANGLE:
	case DRIVER_DATA_NVEH_LATERAL_POSITION:
	case DRIVER_DATA_NVEH_DISTANCE:
		/*For car following calculations*/
		if ((index1 == 0) && (index2 == 1)) {
			relative_distance = double_value;
		}

		/*For lane changing operations */
		if ((index1 == 1) && (index2 == 1)) {
			vehicle_headwy_left_downstrm = double_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			vehicle_headwy_left_upstrm = (double_value);
		}
		if ((index1 == -1) && (index2 == 1)) {
			vehicle_headwy_right_downstrm = (double_value);
		}
		if ((index1 == -1) && (index2 == -1)) {
			vehicle_headwy_right_upstrm = (double_value);
		}
		if ((index1 == 0) && (index2 == -1)) {
			vehicle_headwy_current_upstrm = (double_value);
		}
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_headwy_current_downstrm = (double_value);
		}
		return 1;
	case DRIVER_DATA_NVEH_REL_VELOCITY:
		/*For car following calculations*/
		if ((index1 == 0) && (index2 == 1)) {
			relative_velocity = double_value;
		}

		/*For lane changing operations */
		if ((index1 == 1) && (index2 == 1)) {
			vehicle_rel_spd_left_downstrm = double_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			vehicle_rel_spd_left_upstrm = double_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			vehicle_rel_spd_right_downstrm = double_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			vehicle_rel_spd_right_upstrm = double_value;
		}
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_rel_spd_current_downstrm = double_value;
		}
		if ((index1 == 0) && (index2 == -1)) {
			vehicle_rel_spd_current_upstrm = double_value;
		}
		return 1;
	case DRIVER_DATA_NVEH_ACCELERATION:
		if ((index1 == 0) && (index2 == 1)) {
			leading_veh_acc = double_value;
		}
		return 1;
	case DRIVER_DATA_NVEH_LENGTH:
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_length = double_value;
		}

		/*For lane changing operations */
		if ((index1 == 1) && (index2 == 1)) {
			vehicle_length_left_downstrm = double_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			vehicle_length_left_upstrm = double_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			vehicle_length_right_downstrm = double_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			vehicle_length_right_upstrm = double_value;
		}
		return 1;
	case DRIVER_DATA_NVEH_WIDTH:
	case DRIVER_DATA_NVEH_WEIGHT:
	case DRIVER_DATA_NVEH_TURNING_INDICATOR:
	case DRIVER_DATA_NVEH_CATEGORY:
	case DRIVER_DATA_NVEH_LANE_CHANGE:
	case DRIVER_DATA_NO_OF_LANES:
		number_of_lanes = index2;
		return 1;
	case DRIVER_DATA_LANE_WIDTH:
	case DRIVER_DATA_LANE_END_DISTANCE:
	case DRIVER_DATA_RADIUS:
	case DRIVER_DATA_MIN_RADIUS:
	case DRIVER_DATA_DIST_TO_MIN_RADIUS:
	case DRIVER_DATA_SLOPE:
	case DRIVER_DATA_SLOPE_AHEAD:
	case DRIVER_DATA_SIGNAL_DISTANCE:
	case DRIVER_DATA_SIGNAL_STATE:
	case DRIVER_DATA_SIGNAL_STATE_START:
	case DRIVER_DATA_SPEED_LIMIT_DISTANCE:
	case DRIVER_DATA_SPEED_LIMIT_VALUE:
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:
		desired_acceleration = double_value;
		return 1;
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		desired_lane_angle = double_value;
		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		if (long_value != 0)
			active_lane_change = long_value;
		else
			active_lane_change = long_value;
		vehicleLaneChangeMap[VehicleID] = long_value;

		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		rel_target_lane = long_value;
		return 1;
	default:
		return 0;
	}
}