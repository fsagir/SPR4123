#ifndef H_VEH_DATA_CLASS_H
#define H_VEH_DATA_CLASS_H

//vehicle data structure
#include "Vehicle_Data_Subset.h"

//Include standard Library Assosiative Array (Map)
#include <map>
#include <memory>
#include <string>
#include <random>
#include <chrono>


class Vehicle
{
public:
	// Map containing a list of all vehicles in the simulation, int is used as a Key value, and the
	//	above declared VehicleData structure as the value. Allows for reference.
	static std::map<int, std::unique_ptr<Vehicle>> DataMap;
	//Function to detirmine current SAE level
	static constexpr SAE_MODE_SPEC Current_SAE_Spec();
	static constexpr SAE_MODE Current_SAE();
	static constexpr SAE_MODE_TYPE Current_SAE_TYPE();
	static constexpr int SAE_LEVEL();
	static constexpr const char * SAE_TYPE();
	//Function to update simulation variables from VISSIM
	static void DataSetValue(long, long, long, long, double, char *);

	static SAE_VEHICLE::StaticData Simulation;

	static Vehicle & VehicleNumber(long int);
	static SAE_VEHICLE::VehicleData & VehicleData(long int);


	//Constructor
	Vehicle(long VehicleID);
	Vehicle(int V_id_LU, int V_id_RU, long VehicleID);

	//disable default and copy-contructor (class is non-copiable)
	Vehicle() = delete;
	Vehicle(const Vehicle &) = delete;

	//method to update this vehicle's simulation frame
	void poll();

	// Function to store the individual data of a vehicle (data is passed by reference)
	void StoreVehicleFrame();

	double LateralDeviation();
	//method to determine Lateral Position
	double DetermineLatPosValue();
	//method to determine lane change selection
	long DetermineLaneChangeValue();
	//method to calulate acceleration change
	double CalculateAccChange();

	SAE_VEHICLE::VehicleData & Vehicle_Data();

	//method to reteve vehicle ID value
	long GetVehicleID();
	//method to retreve current control mode
	Control_Mode & Control_Mode();
	//method to retreve reaction time value
	static double reaction_time();
private:
	std::string CurTime();
	//enumeration for control-type values

	//pointer to hold vehicle data on the heap
	std::unique_ptr<SAE_VEHICLE::VehicleData> Veh_Data;

};

constexpr SAE_MODE_SPEC Vehicle::Current_SAE_Spec()
{
	//Default values depend on preprocessor Macro (set in project configuration)
#ifdef SAE0_CAR
	return SAE_0_CAR;
#elif SAE1_CAR
	return SAE_1_CAR;
#elif SAE2_CAR
	return SAE_2_CAR;
#elif SAE3_CAR
	return SAE_3_CAR;
#elif SAE4_CAR
	return SAE_4_CAR;
#elif SAE5_CAR
	return SAE_5_CAR;
#elif SAE0_TRUCK
	return SAE_0_TRUCK;
#elif SAE1_TRUCK
	return SAE_1_TRUCK;
#elif SAE2_TRUCK
	return SAE_2_TRUCK;
#elif SAE3_TRUCK
	return SAE_3_TRUCK;
#elif SAE4_TRUCK
	return SAE_4_TRUCK;
#elif SAE5_TRUCK
	return SAE_5_TRUCK;
#else
#error SAE level not set
#endif 
}
constexpr SAE_MODE Vehicle::Current_SAE()
{
	switch (Current_SAE_Spec())
	{
	case SAE_0_CAR:
	case SAE_0_TRUCK:
		return SAE_0;
	case SAE_1_CAR:
	case SAE_1_TRUCK:
		return SAE_1;
	case SAE_2_CAR:
	case SAE_2_TRUCK:
		return SAE_2;
	case SAE_3_CAR:
	case SAE_3_TRUCK:
		return SAE_3;
	case SAE_4_CAR:
	case SAE_4_TRUCK:
		return SAE_4;
	case SAE_5_CAR:
	case SAE_5_TRUCK:
		return SAE_5;
	default:
		return SAE_ERROR_N;
	}
}
constexpr SAE_MODE_TYPE Vehicle::Current_SAE_TYPE()
{
	switch (Current_SAE_Spec())
	{
	case SAE_0_CAR:
	case SAE_1_CAR:
	case SAE_2_CAR:
	case SAE_3_CAR:
	case SAE_4_CAR:
	case SAE_5_CAR:
		return SAE_CAR;
	case SAE_0_TRUCK:
	case SAE_1_TRUCK:
	case SAE_2_TRUCK:
	case SAE_3_TRUCK:
	case SAE_4_TRUCK:
	case SAE_5_TRUCK:
		return SAE_TRUCK;
	default:
		return SAE_ERROR_V;
	}
}
constexpr int Vehicle::SAE_LEVEL()
{
	return static_cast<int>(Current_SAE());
}
constexpr const char * Vehicle::SAE_TYPE()
{
	static const char * TypeString = ((Current_SAE_TYPE() == SAE_CAR) ? "CAR" : "TRUCK");
	return TypeString;
}

constexpr double SAE_VEHICLE::StaticData::a()
{
	//Default values depend on preprocessor Macro (set in project configuration)
	if (Vehicle::Current_SAE_Spec() == SAE_5_TRUCK)
	{
		return 1.4;
	}
	if (Vehicle::Current_SAE_TYPE() == SAE_CAR)
	{
		return 1.4;
	}
	else //SAE_TRUCK
	{
		return 0.7;
	}
}

constexpr double SAE_VEHICLE::StaticData::b()
{
	return 2.0;
}

constexpr double SAE_VEHICLE::StaticData::p()
{
	return 0.5;
}

constexpr double SAE_VEHICLE::StaticData::time_headway()
{
	//Default values depend on preprocessor Macro (set in project configuration)
	if (Vehicle::Current_SAE_Spec() == SAE_5_TRUCK)
	{
		return 1.5;
	}
	if (Vehicle::Current_SAE_TYPE() == SAE_CAR)
	{
		return 1.5;
	}
	else //SAE_TRUCK
	{
		return 2.0;
	}
}



#endif //H_VEH_DATA_CLASS_H