#pragma once
//Include standard C++ files for Standardized Integer sizes (eg: uint32_t) and dynamic assosiative arrays (Map)
#include <stdint.h>
#include <map>
#include <vector>
enum Level_Shift { Human_Control = 1, Automated_Control = 0 };
//structure to contain per-vehicle data from the simulation

enum SignalState {
	SIGNAL_STATE_RED = 1,
	SIGNAL_STATE_AMBER = 2,
	SIGNAL_STATE_GREEN = 3,
	SIGNAL_STATE_OFF = 6,
};

//structure to contain per-vehicle data from the simulation
class VehicleData
{
	double a;
	double b;
	int curLaneCount;
	double distanceFromSignal;
	long signalState;
	long curLink;
	long curLane;
#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
	bool isOddZoneTransitionActive;
	double desired_velocity_initial;
	double desired_velocity_final;
	double desired_velocity_current;
	std::vector<std::pair<double, double>> oddZones;
	void initOddZones();
#endif
public:
	long VehicleID;
	double lateral_position;
	double desired_lane_angle;
	double Cur_veh_angle;
	double Random_value;
	double current_time;
	double  current_velocity;
	double  current_acceleration;
	double	x_coordinate;
	double  y_coordinate;
	double x_coordinate_rear;
	double relative_distance;
	long Initial_Lane;
	long vehicle_type;
	bool lane_set;
	bool final_link_set;
	bool link_set;
	bool Volume_set;
	long Initial_link;
	long final_link;
	long active_lane_change;
	//Change volume declared as uint32 to allow for large volumes
	uint32_t Change_volume;
	bool Lane_change_in_progress;
	//General traffic volume declared as static to allow for sharing of this variable between all copies of structure
	static uint32_t Volume;
	//Constructor to initialise data - defined here, implemented in DataMap.cpp
	VehicleData();
	VehicleData(const VehicleData & oldObj);
	double LateralDeviation();
	double y1;
	double y2;
	double y3;
	double future_lateral_position;
	Level_Shift level_shift;
	double time_to_shift;
	double Time_of_completion_of_lane_change;
	double Time_of_change_of_control_on_lane_angle;
	bool decided_to_stop_at_signal;
	bool decided_to_yeild;
	double deceleration_at_signal;
	double deceleration_to_yeild;
	long cur_link;
	long cur_veh_lane;
	double y_coordinate_rear;
	double length;
	double width;
	double desired_velocity_previous_time_step;




	double getConstA();
	double getConstB();
	void setLaneCount(int laneNumber);
	int getLaneCount();
	bool isCurLaneLeftMost(long curLane);
	bool isCurLaneRightMost(long curLane);
	void setDistanceFromSignal(double distance);
	double getDistanceFromSignal();
	void setSignalState(long state);
	long getSignalState();
	void setCurLink(long curLink);
	long getCurLink();
	void setCurLane(long curLane);
	long getCurLane();

#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
	void setDesiredVelocityIntial(double value);
	double getDesiredVelocityInitial();
	void setDesiredVelocityFinal(double value);
	double getDesiredVelocityFinal();
	void setDesiredVelocityStep(double value);
	double getDesiredVelocityStep();
	static double getNextDesiredVelocityStep(double velo_initial, double velo_final, double velo_current);
	bool isTransitionOddZoneGoingOn();
	void setTransitionOddZone(bool value);
	bool isCoordinateWithinOddZone(double x, double y);
#endif

	static std::map<long, std::map<long, std::vector<VehicleData>>> getVehicleDistanceFromSignal();
	static std::vector<VehicleData> getVehicleDistanceFromSignal(long linkNumber, long laneNumber);
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