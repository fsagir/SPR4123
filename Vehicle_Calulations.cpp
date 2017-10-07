#include "DriverModel.h"
#include "Vehicle_data.h"

#include <fstream>	//file-stream
#include <ctime>	//time functions
#include <sstream>	//string-stream

#include <windows.h>


double SAE_VEHICLE::VehicleData::DetermineAccValue(double acc_idm, double acc_acc)
{
	if (Current_Level == Human_Control)
	{
		if (Vehicle::Simulation.current_time > time_to_shift)
			return acc_idm;
		else
			return acc_acc;
	}
	else //automated control
	{
		if (Vehicle::Simulation.current_time > time_to_shift)
			return acc_acc;
		else
			return acc_idm;
	}
}
long SAE_VEHICLE::VehicleData::DetermineLaneChangeValue()
{
	if (Current_Level == Human_Control)
	{
		if (Vehicle::Simulation.current_time > time_to_shift)
			return active_lane_change;
		else
			return CalculateAutomatedLaneChange(acc_idm);
	}
	else //(Control_Mode == Automated_Control)
	{
		if (Vehicle::Simulation.current_time > time_to_shift)
			return CalculateAutomatedLaneChange(acc_idm);
		else
			return active_lane_change;
	}
}
double SAE_VEHICLE::VehicleData::ACC_idm(ACC_idm_Data ACC_Data)
{
	double space_headway = ACC_Data.veh_headwy - ACC_Data.veh_length + ACC_Data.veh_rel_spd * time_ln_ch;
	double ratio_upstream_ = ACC_Data.current_vel / ACC_Data.desired_vel;
	double desired_space_headway_ = jam_distance + ACC_Data.current_vel * time_headway + 0.5 * (ACC_Data.current_vel - ACC_Data.veh_rel_spd) * (-ACC_Data.veh_rel_spd) / sqrt(Vehicle::Simulation.a() * Vehicle::Simulation.b());
	space_ratio = desired_space_headway_ / space_headway;
	return Vehicle::Simulation.a() * (1 - pow(ratio_upstream_, 4));
}
long SAE_VEHICLE::VehicleData::CalculateAutomatedLaneChange(double & acc_idm)
{
	lane_change_to_left = 0.0;
	lane_change_to_right = 0.0;

	if (cur_link == 7)
		number_of_lanes = 2;
	else
		number_of_lanes = 1;

	if (number_of_lanes == 1 || Vehicle::VehicleData(vehicle_ID_current_upstrm).Lane_change_in_progress == true
		|| Lane_change_in_progress == true
		|| Vehicle::VehicleData(vehicle_ID_current_upstrm).Lane_change_in_progress == true
		|| Vehicle::VehicleData(vehicle_ID_current_upstrm).Lane_change_in_progress == true)
	{
		return 0;
	}

	if (vehicle_ID_current_upstrm > 0)
	{
		ACC_idm_Data Param;
		Param.veh_headwy = vehicle_headwy_current_downstrm + vehicle_headwy_current_upstrm;
		Param.veh_length = vehicle_length;
		Param.veh_rel_spd = -(vehicle_rel_spd_current_downstrm - vehicle_rel_spd_current_upstrm);
		Param.current_vel = (current_velocity - vehicle_rel_spd_current_upstrm);
		Param.desired_vel = Vehicle::VehicleData(vehicle_ID_current_upstrm).desired_velocity;

		acc_idm_current_upstream = ACC_idm(Param) - pow(space_ratio, 2);
	}
	else acc_idm_current_upstream = 0;

	if (cur_veh_lane != number_of_lanes)
	{
		//leftmost lane

		/*a) Checking for the changed new acceleration of the upstream vehcile on the left*/
		if (vehicle_ID_left_upstrm > 0)
		{
			ACC_idm_Data Param;
			Param.veh_headwy = vehicle_headwy_left_upstrm;
			Param.veh_length = vehicle_length_left_upstrm;
			Param.veh_rel_spd = vehicle_rel_spd_left_upstrm;
			Param.current_vel = (current_velocity - vehicle_rel_spd_left_upstrm);
			Param.desired_vel = Vehicle::VehicleData(vehicle_ID_left_upstrm).desired_velocity;
			acc_idm_left_upstream = ACC_idm(Param) - pow(space_ratio, 2);
		}
		else acc_idm_left_upstream = 0;


		ACC_idm_Data Param_left_downstrm;
		Param_left_downstrm.veh_headwy = vehicle_headwy_left_downstrm;
		Param_left_downstrm.veh_length = vehicle_length_left_downstrm;
		Param_left_downstrm.veh_rel_spd = -vehicle_rel_spd_left_downstrm;
		Param_left_downstrm.current_vel = current_velocity;
		Param_left_downstrm.desired_vel = desired_velocity;

		if (vehicle_ID_left_downstrm == -1)
		{
			acc_idm_left_self = ACC_idm(Param_left_downstrm);
		}
		else
		{
			acc_idm_left_self = ACC_idm(Param_left_downstrm) - pow(space_ratio, 2);
		}

		if (!(acc_idm_left_upstream <= -b_safe || acc_idm_left_self <= -b_safe))
		{
			if (acc_idm_left_self - acc_idm + Vehicle::Simulation.p() * (acc_idm_left_upstream - vehicle_acc_left_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm) >= acc_thr)
			{
				lane_change_to_left = acc_idm_left_self - acc_idm + Vehicle::Simulation.p() * (acc_idm_left_upstream - vehicle_acc_left_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm);
			}
		}
	}
	else
	{
		acc_idm_left_upstream = 0;
		acc_idm_left_self = 0;
		lane_change_to_left = 0;
	}
	if (cur_veh_lane != 1)
	{
		//rightmost lane


		if (vehicle_ID_right_upstrm > 0)
		{
			ACC_idm_Data Param_right_upstrm;
			Param_right_upstrm.veh_headwy = vehicle_headwy_right_upstrm;
			Param_right_upstrm.veh_length = vehicle_length_right_upstrm;
			Param_right_upstrm.veh_rel_spd = vehicle_rel_spd_right_upstrm;
			Param_right_upstrm.current_vel = current_velocity - vehicle_rel_spd_right_upstrm;
			Param_right_upstrm.desired_vel = Vehicle::VehicleData(vehicle_ID_right_upstrm).desired_velocity;;

			acc_idm_right_upstream = ACC_idm(Param_right_upstrm) - pow(space_ratio, 2);
		}
		else acc_idm_right_upstream = 0;


		ACC_idm_Data Param_right_self;
		Param_right_self.veh_headwy = vehicle_headwy_right_downstrm;
		Param_right_self.veh_length = vehicle_length_right_downstrm;
		Param_right_self.veh_rel_spd = -vehicle_rel_spd_right_downstrm;
		Param_right_self.current_vel = current_velocity;
		Param_right_self.desired_vel = desired_velocity;

		if (vehicle_ID_right_downstrm == -1)
		{


			acc_idm_right_self = ACC_idm(Param_right_self);
		}
		else
		{
			acc_idm_right_self = ACC_idm(Param_right_self) - pow(space_ratio, 2);
		}

		if (!(acc_idm_right_upstream <= -b_safe || acc_idm_right_self <= -b_safe))
		{
			if (acc_idm_right_self - acc_idm + Vehicle::Simulation.p() * (acc_idm_right_upstream - vehicle_acc_right_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm) >= acc_thr)
			{
				lane_change_to_right = acc_idm_right_self - acc_idm + Vehicle::Simulation.p() * (acc_idm_right_upstream - vehicle_acc_right_upstrm + acc_idm_current_upstream - vehicle_acc_current_upstrm);

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


	if ((lane_change_to_left > lane_change_to_right) && (lane_change_to_left >= acc_thr))
	{
		return 1;
	}
	else if ((lane_change_to_right > lane_change_to_left) && (lane_change_to_right >= acc_thr))
	{
		return  -1;
	}
	else if ((lane_change_to_right == lane_change_to_left) && (lane_change_to_right > acc_thr))
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
double SAE_VEHICLE::VehicleData::LateralDeviation()
{
	//generate seed value from time
	uint32_t seed = static_cast<uint32_t>(std::chrono::system_clock::now().time_since_epoch().count());
	//seed random generator
	std::default_random_engine generator(seed);
	//create distribution
	std::normal_distribution<double> distribution(0.0, 0.0046);
	//generate value using normal distribution and seeded generator
	double RandomValue = distribution(generator);
	//aquire absolute value
	RandomValue = abs(RandomValue);
	//RandomValue = 0;

	static double y1 = 0;
	static double y2 = 0;
	static double y3 = 0;
	static double future_lateral_position = 0;

	y3 = y2;
	y2 = y1;
	y1 = lateral_position;
	static const double Gamma0 = 0.63;
	static const double Gamma1 = 2.29;
	static const double Beta1 = 0.055;
	static const double Beta2 = 0.47;
	static const double Beta3 = 0.48;
	double W1 = y1;
	double W2 = y1 + (y1 - y3) / 2;
	double W3 = 3 * y1 - 3 * y2 - y3;

	double pt = 1 / (1 + exp(-(Gamma0 + Gamma1*y1)));
	int I_t;
	if (pt > 0.5)
		I_t = -1;
	else
		I_t = 0;

	future_lateral_position = Beta1*W1 + Beta2*W2 + Beta3*W3 + RandomValue * I_t;
	if (lateral_position >= 4 || lateral_position <= -4 || future_lateral_position >= 4 || future_lateral_position <= -4)
		return 0;
	else
		return (future_lateral_position - lateral_position)*2.5 / current_velocity;
}
double SAE_VEHICLE::VehicleData::CalculateAccChange()
{
	/* Calculation of IDM acceleration - SAE Level 0 */
	acc_idm = 0;
	static double acc_acc = 0;
	static double acc_cah = 0;
	space_headway = relative_distance - vehicle_length;
	if (cur_link != 7) {

		desired_velocity = 12.5 + (desired_velocity - 24.44) / 12.5 * (24.44 - 12.5);
	}
	double ratio = current_velocity / desired_velocity;
	desired_space_headway = jam_distance + current_velocity * time_headway +
		0.5 * current_velocity * relative_velocity / sqrt(Vehicle::Simulation.a() * Vehicle::Simulation.b());
	space_ratio = desired_space_headway / space_headway;
	/* checking if the vehicle is the first vehicle*/
	if (vehicle_ID == -1) {
		acc_idm = Vehicle::Simulation.a() * (1 - ratio * ratio * ratio * ratio);
	}
	else {
		acc_idm = Vehicle::Simulation.a() * (1 - ratio * ratio * ratio * ratio - space_ratio * space_ratio);
	}
	if (Vehicle::Current_SAE() == SAE_0)
	{
		return acc_idm;
	}

	/* Calculation of CAH acceleration - will be used to calculate ACC acceleration */
	if (leading_veh_acc > Vehicle::Simulation.a()) {
		effective_acc = Vehicle::Simulation.a();
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
		acc_cah = Vehicle::Simulation.a() * (1 - ratio * ratio * ratio * ratio);
	}
	else if (leading_veh_spd * relative_velocity <= -2 * space_headway * effective_acc) {
		acc_cah = (pow(current_velocity, 2) * effective_acc) / (pow(leading_veh_spd, 2) - 2 * space_headway * effective_acc);
	}
	else {
		acc_cah = effective_acc - (pow(relative_velocity, 2) * heaviside_step) / (2 * space_headway);
	}

	/* Calculation of ACC acceleration - SAE Level 1*/

	if (vehicle_ID == -1) {
		acc_acc = Vehicle::Simulation.a() * (1 - ratio * ratio * ratio * ratio);
	}
	else if (acc_idm >= acc_cah) {
		acc_acc = acc_idm;
	}
	else {

		acc_acc = (1 - c) * acc_idm + c * (acc_cah + Vehicle::Simulation.b() * tanh((acc_idm - acc_cah) / Vehicle::Simulation.b()));

		if (acc_acc < acc_idm) {
			acc_acc = acc_idm;
		}
		/*acc_acc = (1 - c) * acc_idm + c * acc_cah;*/
	}

	//Call to detirmine acceleration 
	return DetermineAccValue(acc_idm, acc_acc);
}
double SAE_VEHICLE::VehicleData::DetermineLatPosValue()
{
	if (Current_Level == Human_Control)
	{
		if (Vehicle::Simulation.current_time > time_to_shift)
			return Random_value = LateralDeviation();
		else
			return Random_value = (LateralDeviation()) / 10;
	}
	else //if (Control_Mode == Automated_Control)
	{
		if (Vehicle::Simulation.current_time > time_to_shift)
			return Random_value = LateralDeviation() / 10;
		else
			return Random_value = LateralDeviation();
	}
}