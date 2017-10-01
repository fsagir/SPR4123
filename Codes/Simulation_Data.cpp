//include header file describing data
#include "Simulation_Data.h"
#include <cmath>
#include <random>
#include <chrono>

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
double  c = 0.99;
double  b_safe = 4.0;
double  time_ln_ch = 1.0;
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
double  p = 0.5;
double  acc_thr = .35;
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
double RandomValue = 0;
//Function to detirmine acceleration value
void DetrimeAccValue(double * double_value)
{
	if ((current_time > DataMap[VehicleID].time_to_shift) && (DataMap[VehicleID].level_shift = 1))
	{
		if ((current_velocity > 15.65) || (x_coordinate > 700 && x_coordinate < 1000))
		{
			if (vehicle_ID == -1) {
				*double_value = acc_idm;
			}
			else {
				*double_value = acc_idm;
			}
		}
	}

	else if ((current_time > DataMap[VehicleID].time_to_shift) && (DataMap[VehicleID].level_shift = 0))
	{
		if ((current_velocity < 15.65))
		{
			if (vehicle_ID == -1) {
				*double_value = acc_acc;
			}
			else {
				*double_value = acc_acc;
			}
		}

		else
		{
			if (vehicle_ID == -1) {
				*double_value = acc_idm;
			}
			else {
				*double_value = acc_idm;
			}
		}
	}
}

/*==========================================================================*/

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
	reaction_time = reaction_time_distribution(generator);


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
		if (double_value > -760 && double_value < 1550)
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
	case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE:
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
			vehicle_headwy_left_upstrm = double_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			vehicle_headwy_right_downstrm = double_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			vehicle_headwy_right_upstrm = double_value;
		}
		if ((index1 == 0) && (index2 == -1)) {
			vehicle_headwy_current_upstrm = double_value;
		}
		if ((index1 == 0) && (index2 == 1)) {
			vehicle_headwy_current_downstrm = double_value;
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
		active_lane_change = long_value;
		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		rel_target_lane = long_value;
		return 1;
	default:
		return 0;
	}
}