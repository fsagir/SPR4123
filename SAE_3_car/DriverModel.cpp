/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Dummy version that simply sends back VISSIM's suggestions to VISSIM.    */
/*                                                                          */
/*  Version of 2010-03-02                                   Lukas Kautzsch  */
/*==========================================================================*/

#include "DriverModel.h"
#include "math.h"

/*==========================================================================*/

double  desired_acceleration = 0.0;
double  desired_lane_angle   = 0.0;
long    active_lane_change   = 0;
long    rel_target_lane      = 0;
double  desired_velocity     = 0.0;
long    turning_indicator    = 0;
long    vehicle_color        = RGB(0,0,0);
double  current_velocity     = 0.0;
double  current_acceleration = 0.0;
double  time_step = 0.0;
double  a = 3.5;
double  b = 3;
double  jam_distance = 2.0;
double  time_headway = 2.0;
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
double  c = 0.99;
double  time_ln_change = 1.0;
/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule, 
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

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type, 
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_PATH                   :
    case DRIVER_DATA_TIMESTEP               :
		time_step = double_value;
    case DRIVER_DATA_TIME                   :
    case DRIVER_DATA_VEH_ID                 :
    case DRIVER_DATA_VEH_LANE               :
    case DRIVER_DATA_VEH_ODOMETER           :
    case DRIVER_DATA_VEH_LANE_ANGLE         :
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
    case DRIVER_DATA_VEH_VELOCITY           :
		current_velocity = double_value;
    case DRIVER_DATA_VEH_ACCELERATION       :
		current_acceleration = double_value;
    case DRIVER_DATA_VEH_LENGTH             :
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = long_value;
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      desired_velocity = double_value;
      return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
    case DRIVER_DATA_VEH_Y_COORDINATE       :
    case DRIVER_DATA_VEH_TYPE               :
      return 1;
    case DRIVER_DATA_VEH_COLOR              :
      vehicle_color = long_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
      return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_NVEH_ID                :
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_ID = long_value;
		}
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
    case DRIVER_DATA_NVEH_DISTANCE          :
		if ((index1 == 0) && (index2 == 1)) {
			relative_distance = double_value;
		}
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
		if ((index1 == 0) && (index2 == 1)) {
			relative_velocity = double_value;
		}

    case DRIVER_DATA_NVEH_ACCELERATION      :
		if ((index1 == 0) && (index2 == 1)) {
			leading_veh_acc = double_value;
		}
    case DRIVER_DATA_NVEH_LENGTH            :
    	if ((index1 == 0) && (index2 == 1)) {
			vehicle_length = double_value;
		}
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      desired_lane_angle = double_value;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      active_lane_change = long_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      rel_target_lane = long_value;
      return 1;
    default :
      return 0;
  }
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

		if (vehicle_ID == -1) {
			*double_value = acc_idm;
		}
		else {
			*double_value = acc_idm;
		}
		return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *long_value = active_lane_change;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      *long_value = rel_target_lane;
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

