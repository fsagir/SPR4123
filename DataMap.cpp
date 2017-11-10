//include driver model and vehicle data header files
#include "Vehicle_data.h"
#include "DriverModel.h"
#include "Simulation_Data.h"
//include standard C++ library files for:
#include <iostream>	//Console input-output (terminal)
#include <fstream>	//File access
#include <string>	//string use
#include <sstream>	//string-stream (used for data conversion)
#include <ctime>	//time functions
#include <random>
#include <chrono>


//Implemention of extern variable in Vehicle_data.h
std::map<int, VehicleData> DataMap; //This stores the vehicle data


//Datasetvalue function implementation (details can be found in Vehicle_data.h)
void DataSetValue(long   type,
	long   index1,
	long   index2,
	long   long_value,
	double double_value,
	char   *string_value)
{
	//This value is the vehicleID
	static long VehicleID = 0;

	//For each peice of vehicle data provided by visim (often in real time), update our assosiative array
	//	with that information.
	//	ie: Store the state of every vehicle in the simulation inside the `DataMap` array.
	switch (type)
	{
	case DRIVER_DATA_VEH_ID:
		VehicleID = long_value;
		DataMap[VehicleID].VehicleID = long_value;
		break;
	case DRIVER_DATA_TIME:
		DataMap[VehicleID].current_time = double_value;
#if defined(SAE5_CAR) || defined(SAE5_TRUCK)
		DataMap[VehicleID].level_shift = Automated_Control;
		break;
#endif
		if ((x_coordinate > -6400) && (x_coordinate < -5600))
		{
			DataMap[VehicleID].level_shift = Human_Control;
			break;
		}
#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
		else
		{
			DataMap[VehicleID].level_shift = Automated_Control;
			break;
		}
#else
		if (DataMap[VehicleID].level_shift == Automated_Control && DataMap[VehicleID].current_velocity > 17.9)
		{
			DataMap[VehicleID].time_to_shift = DataMap[VehicleID].current_time + reaction_time;
			DataMap[VehicleID].level_shift = Human_Control;
		}
		//check to see if we are under human control and below 17.9 m/s speed limit
		if (DataMap[VehicleID].level_shift == Human_Control && DataMap[VehicleID].current_velocity < 17.9)
		{
			DataMap[VehicleID].time_to_shift = DataMap[VehicleID].current_time + 2 * reaction_time;
			DataMap[VehicleID].level_shift = Automated_Control;

		}
#endif

		break;
	case DRIVER_DATA_VEH_VELOCITY:
		DataMap[VehicleID].current_velocity = double_value;
		break;
	case DRIVER_DATA_VEH_ACCELERATION:
		DataMap[VehicleID].current_acceleration = double_value;
		break;
	case DRIVER_DATA_VEH_X_COORDINATE:
		DataMap[VehicleID].x_coordinate = double_value;
		break;
	case DRIVER_DATA_VEH_Y_COORDINATE:
		DataMap[VehicleID].y_coordinate = double_value;
		break;
	case DRIVER_DATA_NVEH_DISTANCE:
		/*For car following calculations*/
		if ((index1 == 0) && (index2 == 1)) {
			DataMap[VehicleID].relative_distance = double_value;
		}
		else
		{
			DataMap[VehicleID].relative_distance = -1;
		}
		break;
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		desired_angle = double_value;
		
		DataMap[VehicleID].desired_lane_angle = double_value;
		break;
	case DRIVER_DATA_VEH_LATERAL_POSITION:
		DataMap[VehicleID].lateral_position = double_value;
		break;
	case DRIVER_DATA_VEH_LANE:
		if (DataMap[VehicleID].lane_set == false)
		{
			DataMap[VehicleID].Initial_Lane = long_value;
			DataMap[VehicleID].lane_set = true;
		}

		break;
	case DRIVER_DATA_VEH_CURRENT_LINK:
		if (DataMap[VehicleID].link_set == false)
		{
			DataMap[VehicleID].Initial_link = long_value;
			DataMap[VehicleID].link_set = true;
		}
		if (DataMap[VehicleID].final_link_set == false && DataMap[VehicleID].y_coordinate <= 124)
		{
			DataMap[VehicleID].final_link = long_value;
			DataMap[VehicleID].final_link_set = true;
		}

		break;
	case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE:
		if (long_value != 0)
		{
			if (DataMap[VehicleID].Lane_change_in_progress == false)
			{
				DataMap[VehicleID].Change_volume = 1;
			}
			else
			{
				DataMap[VehicleID].Change_volume = 0;
			}
			DataMap[VehicleID].Lane_change_in_progress = true;
		}
		else
		{   
			if (DataMap[VehicleID].Lane_change_in_progress == true)
			{
				DataMap[VehicleID].Time_of_completion_of_lane_change = current_time;
				DataMap[VehicleID].Time_of_change_of_control_on_lane_angle = DataMap[VehicleID].Time_of_completion_of_lane_change + duration_of_vissim_conrol_on_angle_after_lane_change;

			}

			DataMap[VehicleID].Lane_change_in_progress == false;
			
		}
	case DRIVER_DATA_VEH_LANE_ANGLE:
		DataMap[VehicleID].Cur_veh_angle = double_value;
		break;
	case DRIVER_DATA_VEH_TYPE:
		DataMap[VehicleID].vehicle_type = long_value;
		break;
	}
}

//current time function used to create file-name using current time date (as a string)
std::string CurTime()
{
	time_t rawtime;
	struct tm * timeinfo;
	char buffer[80];
	time(&rawtime);
	timeinfo = localtime(&rawtime);
	strftime(buffer, sizeof(buffer), "%d-%m-%Y_%I-%M", timeinfo);
	std::string retval(buffer);
	return retval;
}
//store data for current vehicle at current time, details in Vehicle_data.h
//This outputs one vehicle to the CSV file.
void StoreVehicleFrame(VehicleData & DataPoint)
{
	//Generate file name (only occours once)
#ifdef SAE0_CAR
	static const char * data = "SAE0_CAR_";
	static const char * FilePathCombiled = "SAE0_COMBINED_";
#elif SAE1_CAR
	static const char * data = "SAE1_CAR_";
	static const char * FilePathCombiled = "SAE1_COMBINED_";
#elif SAE2_CAR
	static const char * data = "SAE2_CAR_";
	static const char * FilePathCombiled = "SAE2_COMBINED_";
#elif SAE3_CAR
	static const char * data = "SAE3_CAR_";
	static const char * FilePathCombiled = "SAE3_COMBINED_";
#elif SAE4_CAR
	static const char * data = "SAE4_CAR_";
	static const char * FilePathCombiled = "SAE4_COMBINED_";
#elif SAE5_CAR
	static const char * data = "SAE5_CAR_";
	static const char * FilePathCombiled = "SAE5_COMBINED_";
#elif SAE0_TRUCK
	static const char * data = "SAE0_TRUCK_";
	static const char * FilePathCombiled = "SAE0_COMBINED_";
#elif SAE1_TRUCK
	static const char * data = "SAE1_TRUCK_";
	static const char * FilePathCombiled = "SAE1_COMBINED_";
#elif SAE2_TRUCK
	static const char * data = "SAE2_TRUCK_";
	static const char * FilePathCombiled = "SAE2_COMBINED_";
#elif SAE3_TRUCK
	static const char * data = "SAE3_TRUCK_";
	static const char * FilePathCombiled = "SAE3_COMBINED_";
#elif SAE4_TRUCK
	static const char * data = "SAE4_TRUCK_";
	static const char * FilePathCombiled = "SAE4_COMBINED_";
#elif SAE5_TRUCK
	static const char * data = "SAE5_TRUCK_";
	static const char * FilePathCombiled = "SAE5_COMBINED_";

#else
#error SAE LEVEL NOT SET
#endif
	{
		static std::string filename = "C:\\DataLog\\" + std::string(data) + '_' + CurTime() + ".csv";
		static std::ofstream Datafile(filename, std::ios_base::out);
		static std::string filename2 = "C:\\DataLog\\" + std::string(FilePathCombiled) + '_' + CurTime() + ".csv";
		static std::ofstream Datafile2(filename2, std::ios_base::out | std::ios_base::ate | std::ios_base::app);
		//Ignore all vehicles that start in the rightmost lane
		/*if (DataPoint.Initial_link != 8) return;*/

		//The following code is used for debugging file-output if we "Cannot create or open" the file
		if (Datafile.fail() || Datafile2.fail())
		{

			if (GetLastError() != 0)
			{
				std::string result;
				char Buffer[256];
				FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM, 0, GetLastError(), 0, Buffer, 255, 0);
				result = std::string(Buffer);
			}
			return;
		}


		//begin outputting the vehicle data into a `stringstream` object (converts all information into
		//	a plain-text format.

		std::stringstream SS;
		SS << DataPoint.VehicleID << ','
			<< DataPoint.vehicle_type << ','
			<< DataPoint.Initial_link << ','
			<< DataPoint.Initial_Lane << ','
			<< DataPoint.final_link << ','
			<< DataPoint.current_time << ','
			<< DataPoint.current_acceleration << ','
			<< DataPoint.current_velocity << ','
			<< DataPoint.relative_distance << ','
			<< DataPoint.x_coordinate << ','
			<< DataPoint.y_coordinate << ','
			<< VehicleData::Volume << ','
			<< DataPoint.lateral_position << ','
			<< DataPoint.desired_lane_angle << ','
			//		<< DataPoint.Cur_veh_angle << ','
			<< DataPoint.Random_value << ','
			<< DataPoint.Change_volume

			<< '\n';
		//Take resulting stream and save to a single String.
		std::string output = SS.str();
		//output that string into our data-collection files 
		Datafile << output;
		Datafile2 << output;
		Datafile2.flush();


		



	}


}

void StoreSituationData(int Veh)
{
	static std::string Time = CurTime();
	std::string data = "Vehicle";
	data += std::to_string(Veh) + "_";
	std::string filename3 = "C:\\DataLog\\" + std::string(data) + "_DEBUG_" + Time + ".csv";
	std::ofstream Datafile3(filename3, std::ios_base::out | std::ios_base::app);
	std::stringstream SS;
	SS.str("");
	SS << VehicleID << ','
		<< x_coordinate << ','
		<< current_velocity << ','
		<< desired_velocity << ','
		<< pow(ratio,4) << ','
		<< pow(space_ratio,2) << ','
		<< acc_acc << ','
		<< acc_idm << ','
		<< desired_space_headway << ','
		<< relative_distance << ','
		<< vehicle_ID << ','
		<< DataMap[vehicle_ID].current_velocity << ','
		<< "\n";

	std::string output = SS.str();
	Datafile3 << output;
	Datafile3.flush();
}


//Implementation of constructor
VehicleData::VehicleData() : Initial_Lane(0), Change_volume(0), lane_set(false), Volume_set(false), link_set(false), final_link_set(false), Initial_link(0), final_link(0), Lane_change_in_progress(false)
{
	y1 = 0;
	y2 = 0;
	y3 = 0;
	future_lateral_position = 0;
	level_shift = Human_Control;
	time_to_shift= 0.0;
	Time_of_completion_of_lane_change =0.0;
	Time_of_change_of_control_on_lane_angle =0.0;
};
//initialisation of static-volume property to zero.
uint32_t VehicleData::Volume = 0;

double VehicleData::LateralDeviation()
{

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
	//RandomValue = 0;

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
	if (lateral_position >= 1.5|| lateral_position <= -1.5 || future_lateral_position >=1.5 || future_lateral_position <= -1.5)
		return lateral_position;
	/*	future_lateral_position = abs((Beta1*W1 + Beta2*W2 + Beta3*W3 + RandomValue * I_t)*10/current_velocity);*/
	else 
		return future_lateral_position;
	//return future_lateral_position;// / (current_velocity * 10000000000000000000);
	//return 0;
}