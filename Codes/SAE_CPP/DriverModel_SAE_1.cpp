/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that simply sends back VISSIM's suggestions to VISSIM.    */
/*                                                                          */
/*  Version of 2010-03-02                                   Lukas Kautzsch  */
/*==========================================================================*/

#include "DriverModel.h"
#include "Vehicle_data.h"
#include "Simulation_Data.h"
#include <cmath>
#include <random>
#include <chrono>


///*==========================================================================*/

BOOL APIENTRY DllMain(HANDLE  hModule,
	DWORD   ul_reason_for_call,
	LPVOID  lpReserved)
{
	switch (ul_reason_for_call) {
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}
	return TRUE;
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue(long   type,
	long   index1,
	long   index2,
	long   *long_value,
	double *double_value,
	char   **string_value)
{
	/* Gets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, and writes that value to <*double_value>,     */
	/* <*float_value> or <**string_value> (object and value selection       */
	/* depending on <type>).                                                */
	/* Return value is 1 on success, otherwise 0.                           */

	//generate seed value from time
	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	//seed random generator
	std::default_random_engine generator(seed);
	//create distribution
	std::normal_distribution<double> distribution(0.0, 0.0046);
	
	//generate value using normal distribution and seeded generator
	double RandomValue = distribution(generator);

	//aquire absolute value
	RandomValue = abs(RandomValue);

	switch (type) {
	case DRIVER_DATA_STATUS:
		*long_value = 0;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		*long_value = turning_indicator;
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		/* *double_value = current_velocity + current_acceleration * time_step; */
		*double_value = desired_velocity;
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		*long_value = vehicle_color;
		return 1;
	case DRIVER_DATA_WANTS_SUGGESTION:
		*long_value = 1;
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:

		/* Calculation of IDM acceleration - SAE Level 0 */

		space_headway = relative_distance - vehicle_length;
		if (cur_link != 7) {

			desired_velocity = 45 + (desired_velocity - 88) / 42 * 10;
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




		//if ((current_velocity > 15.65) || (x_coordinate > 700 && x_coordinate <1000)) && current_time > time_to_shift && level_shift = 1 ) {

		//	if (vehicle_ID == -1) {
		//		*double_value = acc_idm;
		//	}
		//	else {
		//		*double_value = acc_idm;
		//	}
		//}

		//else { 
		//	if (vehicle_ID == -1) {
		//		*double_value = acc_acc;
		//	}
		//	else {
		//		*double_value = acc_acc;
		//	}
		//}

		return 1;
		//case DRIVER_DATA_DESIRED_LANE_ANGLE:
		//	*double_value = desired_lane_angle;
		//	return 1;
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		/*RandomValue *= 100;*/
		*double_value = DataMap[VehicleID].Random_value = DataMap[VehicleID].LateralDeviation();
		//if (desired_lane_angle >= 0)
		//{
		//	*double_value = RandomValue;
		//}
		//else
		//{
		//	*double_value = -RandomValue;

		//}
		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		*long_value = active_lane_change;
		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		*long_value = rel_target_lane;
		return 1;
	case DRIVER_DATA_SIMPLE_LANECHANGE:
		*long_value = 1;
		return 1;
	default:
		return 0;
	}
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand(long number)
{
	/* Executes the command <number> if that is available in the driver */
	/* module. Return value is 1 on success, otherwise 0.               */

	switch (number) {
	case DRIVER_COMMAND_INIT:
		return 1;
	case DRIVER_COMMAND_CREATE_DRIVER:
		return 1;
	case DRIVER_COMMAND_KILL_DRIVER:
		return 1;
	case DRIVER_COMMAND_MOVE_DRIVER:
		return 1;
	default:
		return 0;
	}
}

/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/

