/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that simply sends back VISSIM's suggestions to VISSIM.    */
/*                                                                          */
/*  Version of 2010-03-02                                   Lukas Kautzsch  */
/*==========================================================================*/

#include "DriverModel.h"
#include <cmath>
#include "Vehicle_data.h"
#include "IDM_acc.h"
#include <vector>
#include "Simulation_Data.h"
#include <random>
#include <chrono>

/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule, 
                       DWORD   ul_reason_for_call, 
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
		  vehicle_desired_vel_array = std::vector<double>(3601, 27.7);
		  vehicle_X_coordinate_array = std::vector<double>(3601, -1000);
		  break;
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }


  return TRUE;
}



/*--------------------------------------------------------------------------*/
DRIVERMODEL_API  int  DriverModelGetValue (long   type, 
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

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	//seed random generator
	std::default_random_engine generator(seed);
	//create distribution
	std::normal_distribution<double> distribution(0.0, 0.00046);
	//generate value using normal distribution and seeded generator
	double RandomValue = distribution(generator);
	//aquire absolute value
	RandomValue = abs(RandomValue);


  switch (type) {
    case DRIVER_DATA_STATUS :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *long_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
     /* *double_value = current_velocity + current_acceleration * time_step; */
		*double_value = desired_velocity;
      return 1;
    case DRIVER_DATA_VEH_COLOR :
		*long_value = vehicle_color;
      return 1;
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 1;
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
    	
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
			acc_cah = ( pow(current_velocity,2) * effective_acc )/( pow(leading_veh_spd,2) - 2 * space_headway * effective_acc);
		}
		else {
			acc_cah = effective_acc - ( pow(relative_velocity,2) * heaviside_step ) / ( 2 * space_headway);
		}

		/* Calculation of ACC acceleration - SAE Level 1*/

		if (vehicle_ID == -1) {
			acc_acc = a * (1 - ratio * ratio * ratio * ratio);
		}
		else if (acc_idm >= acc_cah) {
			acc_acc = acc_idm;
		}
		else {
			acc_acc = (1-c) * acc_idm + c * (acc_cah + b * tanh((acc_idm - acc_cah)/b));
		}

	    *double_value = acc_acc;
		
		return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
		//if (current_veh_angle >= 0)
		//{
		//	*double_value = RandomValue;
		//}
		//else
		//{
		//	*double_value = -RandomValue;

		//}
		*double_value = DataMap[VehicleID].Random_value = (DataMap[VehicleID].LateralDeviation()) / 10;

		return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
		lane_change_to_left = 0.0;
		lane_change_to_right = 0.0;
		*long_value = 0l;
		if(cur_link==7)
			number_of_lanes = 2;
		else
			number_of_lanes = 1;

		if (number_of_lanes == 1 || DataMap[vehicle_ID_current_upstrm].Lane_change_in_progress == true
			|| DataMap[vehicle_ID].Lane_change_in_progress == true 
			|| DataMap[vehicle_ID_current_two_upstream].Lane_change_in_progress == true
			|| DataMap[vehicle_ID_current_two_downstream].Lane_change_in_progress == true)
		{
			*long_value = 0;
			return 1;
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
			acc_idm_current_upstream = ACC_idm(vehicle_headwy_current_downstrm + vehicle_headwy_current_upstrm, vehicle_length, -(vehicle_rel_spd_current_downstrm - vehicle_rel_spd_current_upstrm), time_ln_ch,
			(current_velocity - vehicle_rel_spd_current_upstrm), vehicle_desired_vel_array[vehicle_ID_current_upstrm],
				jam_distance, time_headway, a, b, space_ratio) - pow(space_ratio, 2);
		else acc_idm_current_upstream = 0;

		if (cur_veh_lane != number_of_lanes /*&& DataMap[vehicle_ID_left_upstrm].Lane_change_in_progress == false
			&& DataMap[vehicle_ID_left_downstrm].Lane_change_in_progress == false*/)
		{
			//leftmost lane

			/*a) Checking for the changed new acceleration of the upstream vehcile on the left*/
			if (vehicle_ID_left_upstrm > 0)
				acc_idm_left_upstream = ACC_idm(vehicle_headwy_left_upstrm, vehicle_length_left_upstrm, vehicle_rel_spd_left_upstrm,
					time_ln_ch, (current_velocity - vehicle_rel_spd_left_upstrm), vehicle_desired_vel_array[vehicle_ID_left_upstrm], jam_distance, time_headway, a, b, space_ratio)
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
					time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, a, b, space_ratio);
			}
			else {
				acc_idm_left_self = ACC_idm(vehicle_headwy_left_downstrm, vehicle_length_left_downstrm, -vehicle_rel_spd_left_downstrm,
					time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, a, b, space_ratio) - pow(space_ratio, 2);
			}

			if (!(acc_idm_left_upstream <= -b_safe || acc_idm_left_self <= -b_safe ))
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
		if (cur_veh_lane != 1 /*&& DataMap[vehicle_ID_right_upstrm].Lane_change_in_progress == false
			&& DataMap[vehicle_ID_right_downstrm].Lane_change_in_progress == false*/)
		{
			//rightmost lane

			/*d) Checking for the changed new acceleration of the upstream vehcile on theright*/
			//space_headway_right_upstream = vehicle_headwy_right_upstrm - vehicle_length_right_upstrm + vehicle_rel_spd_right_upstrm * time_ln_ch;
			//ratio_right_upstream = (current_velocity - vehicle_rel_spd_right_upstrm) / vehicle_ID_array[vehicle_ID_right_upstrm];
			//desired_space_headway = jam_distance + (current_velocity - vehicle_rel_spd_right_upstrm) * time_headway + 0.5 * (current_velocity - vehicle_rel_spd_right_upstrm) * (-vehicle_rel_spd_right_upstrm) / sqrt(a * b);
			//space_ratio = desired_space_headway / space_headway_right_upstream;

			if (vehicle_ID_right_upstrm > 0)
				acc_idm_right_upstream = ACC_idm(vehicle_headwy_right_upstrm, vehicle_length_right_upstrm, vehicle_rel_spd_right_upstrm,
					time_ln_ch, (current_velocity - vehicle_rel_spd_right_upstrm), vehicle_desired_vel_array[vehicle_ID_right_upstrm], jam_distance, time_headway, a, b, space_ratio)
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
					time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, a, b, space_ratio);
			}
			else {
				acc_idm_right_self = ACC_idm(vehicle_headwy_right_downstrm, vehicle_length_right_downstrm, -vehicle_rel_spd_right_downstrm,
					time_ln_ch, current_velocity, desired_velocity, jam_distance, time_headway, a, b, space_ratio) - pow(space_ratio, 2);
			}

			if (!(acc_idm_right_upstream <= -b_safe || acc_idm_right_self <= -b_safe ))
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
			*long_value = 1;
		}
		else if ((lane_change_to_right > lane_change_to_left) && (lane_change_to_right >= acc_thr)) {
			*long_value = -1;
		}
		else if ((lane_change_to_right == lane_change_to_left) && (lane_change_to_right > acc_thr)) {
			*long_value = 1;
		}
		else {
			*long_value = 0;
		}
		//*long_value = (rand() % 3) - 1;
    
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
		/**long_value = (rand() % 3) - 1;*/
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *long_value = 1;
      return 1;
    default :
      return 0;
  }
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */
  
  switch (number) {
    case DRIVER_COMMAND_INIT :
      return 1;
    case DRIVER_COMMAND_CREATE_DRIVER :
      return 1;
    case DRIVER_COMMAND_KILL_DRIVER :
      return 1;
    case DRIVER_COMMAND_MOVE_DRIVER :
      return 1;
    default :
      return 0;
  }
}

/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/

