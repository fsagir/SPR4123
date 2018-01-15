#pragma once
//Include standard C++ files for Standardized Integer sizes (eg: uint32_t) and dynamic assosiative arrays (Map)
#include <stdint.h>
#include <map>
enum Level_Shift { Human_Control = 1, Automated_Control = 0 };
//structure to contain per-vehicle data from the simulation
class VehicleData
{
public:
	long VehicleID;
	long active_lane_change;
	double lateral_position;
	double desired_lane_angle;
	double Cur_veh_angle;
	double Random_value;
	double current_time;
	double  current_velocity;
	double  current_acceleration;
	double	x_coordinate;
	double  y_coordinate;
	double relative_distance;
	long Initial_Lane;
	long vehicle_type;
	bool lane_set;
	bool final_link_set;
	bool link_set;
	bool Volume_set;
	long Initial_link;
	long final_link;
	//Change volume declared as uint32 to allow for large volumes
	uint32_t Change_volume;
	bool Lane_change_in_progress;
	//General traffic volume declared as static to allow for sharing of this variable between all copies of structure
	static uint32_t Volume;
	//Constructor to initialise data - defined here, implemented in DataMap.cpp
	VehicleData();
	double LateralDeviation();
	void update_stochastic();
	double y1;
	double y2;
	double y3;
	double future_lateral_position;
	Level_Shift level_shift;
	double time_to_shift;
	double Time_of_completion_of_lane_change;
	double Time_of_change_of_control_on_lane_angle;
	//Stochastic experiment variables
	double a_copy;
	double b_copy;
	double time_headway_copy;
	double reaction_time_copy;
	double acc_thr_human_copy;
	double acc_thr_system_copy;

	
	
};
//Extern Map containing a list of all vehicles in the simulation, int is used as a Key value, and the
//	above declared VehicleData structure as the value. Allows for reference.
extern std::map<int, VehicleData> DataMap;
//Example usage of a map
//std::map<float, std::string> data;
//void func()
//{
//	data[0.32] = "red";
//	data[1.1] = "red";
//	data[2] = "blue";
//}

// Function to store the individual data of a vehicle (data is passed by reference)
void StoreVehicleFrame(VehicleData & DataPoint);
void StoreSituationData(int Veh);

//Function to update simulation variables from VISSIM
void DataSetValue(long   type,
	long   index1,
	long   index2,
	long   long_value,
	double double_value,
	char   *string_value);