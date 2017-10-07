#include "DriverModel.h"
#include "Vehicle_data.h"

#include <fstream>	//file-stream
#include <ctime>	//time functions
#include <sstream>	//string-stream

#include <windows.h>

Vehicle * SAE_VEHICLE::StaticData::Cur_Vehicle;
SAE_VEHICLE::VehicleData * SAE_VEHICLE::StaticData::Cur_VehData;

SAE_VEHICLE::StaticData Vehicle::Simulation;
std::map<int, std::unique_ptr<Vehicle>> Vehicle::DataMap; //This stores the vehicle data

Vehicle::Vehicle(long VehicleID) : Veh_Data(new SAE_VEHICLE::VehicleData), Vehicle_Data(*Veh_Data)
{

}
Vehicle::Vehicle(int V_id_LU, int V_id_RU, long VehicleID) : Vehicle(VehicleID)
{

}
double Vehicle::CalculateAccChange()
{
	return Vehicle_Data.CalculateAccChange();
}
long Vehicle::GetVehicleID()
{
	return Vehicle_Data.VehicleID;
}
Control_Mode & Vehicle::Control_Mode()
{
	return Vehicle_Data.Current_Level;
}
double Vehicle::reaction_time()
{
	//generate seed value from time
	static unsigned int seed = static_cast<unsigned int> (std::chrono::system_clock::now().time_since_epoch().count());
	//seed random generator
	static std::default_random_engine generator(seed);
	static std::lognormal_distribution<double> reaction_time_distribution(0.83, 0.53);
	return reaction_time_distribution(generator);
}
std::string Vehicle::CurTime()
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

void Vehicle::DataSetValue(long type, long index1, long index2, long long_value, double double_value, char * string_value)
{

	//Simple accessers for active vehicle data
	static Vehicle * & Cur_Vehicle = Vehicle::Simulation.Cur_Vehicle;
	static SAE_VEHICLE::VehicleData * & Cur_VehData = Vehicle::Simulation.Cur_VehData;
	//	For each peice of vehicle data provided by visim (often in real time), update our assosiative array
	//	with that information. ie: Store the state of every vehicle in the simulation inside the `DataMap` array.
	switch (type)
	{

	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		Cur_VehData->active_lane_change = long_value;
		break;
	case DRIVER_DATA_REL_TARGET_LANE:
		Cur_VehData->rel_target_lane = long_value;
		break;
	case DRIVER_DATA_DESIRED_ACCELERATION:
		Cur_VehData->desired_acceleration = double_value;
		break;
	case DRIVER_DATA_NO_OF_LANES:
		Cur_VehData->number_of_lanes = index2;
		break;
	case DRIVER_DATA_NVEH_ACCELERATION:
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->leading_veh_acc = double_value;
		}
		break;
	case DRIVER_DATA_VEH_COLOR:
		Cur_VehData->vehicle_color = long_value;
		break;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		Cur_VehData->desired_velocity = double_value;
		break;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		Cur_VehData->turning_indicator = long_value;
		break;
	case DRIVER_DATA_TIMESTEP:
		Cur_VehData->time_step = double_value;
		break;
	case DRIVER_DATA_VEH_ID:
		Vehicle::Simulation.Current_Vehicle_ID = long_value;
		//if this vehicle does not yet exist in the data-map
		if (DataMap.find(Vehicle::Simulation.Current_Vehicle_ID) == DataMap.end())
		{
			//Generate a new vehicle object and move it into the datamap (vehicle is a noncopiable type)
			std::unique_ptr<Vehicle> VPtr(new Vehicle(Vehicle::Simulation.Current_Vehicle_ID));
			DataMap.emplace(std::make_pair(Vehicle::Simulation.Current_Vehicle_ID, std::move(VPtr)));
		}

		Cur_Vehicle = DataMap[Vehicle::Simulation.Current_Vehicle_ID].get();
		Cur_VehData = & Cur_Vehicle->Vehicle_Data;
		break;
	case DRIVER_DATA_TIME:
		Vehicle::Simulation.current_time = double_value;
		for (auto &T : DataMap)
			T.second->poll();
		break;
	case DRIVER_DATA_VEH_VELOCITY:
		Cur_VehData->current_velocity = double_value;
		Cur_VehData->current_velocity = double_value;
		break;
	case DRIVER_DATA_VEH_ACCELERATION:
		Cur_VehData->current_acceleration = double_value;
		Cur_VehData->current_acceleration = double_value;
		break;
	case DRIVER_DATA_VEH_X_COORDINATE:
		//if (double_value > -760 && double_value < 1550)
		Cur_Vehicle->StoreVehicleFrame();
		Cur_VehData->x_coordinate = double_value;
		break;
	case DRIVER_DATA_VEH_Y_COORDINATE:
		Cur_VehData->y_coordinate = double_value;
		break;
	case DRIVER_DATA_NVEH_ID:
		/*To identify if any vehicle is in front of the current vehicle*/
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->vehicle_ID = long_value;
		}
		/*storing vehcile id for lane changing operations*/
		if ((index1 == 1) && (index2 == 1)) {
			Cur_VehData->vehicle_ID_left_downstrm = long_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			Cur_VehData->vehicle_ID_left_upstrm = long_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			Cur_VehData->vehicle_ID_right_downstrm = long_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			Cur_VehData->vehicle_ID_right_upstrm = long_value;
		}
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->vehicle_ID_current_downstrm = long_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			Cur_VehData->vehicle_ID_current_upstrm = long_value;
		}
		if ((index1 == 0) && (index2 == 2)) {
			Cur_VehData->vehicle_ID_current_two_downstream = long_value;
		}
		if ((index1 == 0) && (index2 == -2)) {
			Cur_VehData->vehicle_ID_current_two_upstream = long_value;
		}
		break;
	case DRIVER_DATA_NVEH_DISTANCE:
		/*For car following calculations*/
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->relative_distance = double_value;
		}
		else
		{
			Cur_VehData->relative_distance = -1;
		}
		/*For car following calculations*/
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->relative_distance = double_value;
		}

		/*For lane changing operations */
		if ((index1 == 1) && (index2 == 1)) {
			Cur_VehData->vehicle_headwy_left_downstrm = double_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			Cur_VehData->vehicle_headwy_left_upstrm = double_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			Cur_VehData->vehicle_headwy_right_downstrm = double_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			Cur_VehData->vehicle_headwy_right_upstrm = double_value;
		}
		if ((index1 == 0) && (index2 == -1)) {
			Cur_VehData->vehicle_headwy_current_upstrm = double_value;
		}
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->vehicle_headwy_current_downstrm = double_value;
		}

		break;
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		Cur_VehData->desired_lane_angle = double_value;
		break;
	case DRIVER_DATA_VEH_LATERAL_POSITION:
		Cur_VehData->lateral_position = double_value;
		Cur_VehData->lateral_position = double_value;
		break;
	case DRIVER_DATA_VEH_LANE:
		Cur_VehData->cur_veh_lane = long_value;
		if (Cur_VehData->Initial_Lane == -1)
			Cur_VehData->Initial_Lane = long_value;

		break;
	case DRIVER_DATA_VEH_CURRENT_LINK:
		Cur_VehData->cur_link = long_value;
		if (Cur_VehData->Initial_link == -1)
			Cur_VehData->Initial_link = long_value;

		if (Cur_VehData->final_link_set == false
			&& Cur_VehData->y_coordinate <= 124)
		{
			Cur_VehData->final_link = long_value;
			Cur_VehData->final_link_set = true;
		}

		break;
	case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE:
		if (long_value == 1)
		{
			if (Cur_VehData->Lane_change_in_progress == false)
			{
				Cur_VehData->Change_volume = 1;
			}
			else
			{
				Cur_VehData->Change_volume = 0;
			}
			Cur_VehData->Lane_change_in_progress = true;
		}
		else Cur_VehData->Lane_change_in_progress = false;
	case DRIVER_DATA_NVEH_REL_VELOCITY:
		/*For car following calculations*/
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->relative_velocity = double_value;
		}

		/*For lane changing operations */
		if ((index1 == 1) && (index2 == 1)) {
			Cur_VehData->vehicle_rel_spd_left_downstrm = double_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			Cur_VehData->vehicle_rel_spd_left_upstrm = double_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			Cur_VehData->vehicle_rel_spd_right_downstrm = double_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			Cur_VehData->vehicle_rel_spd_right_upstrm = double_value;
		}
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->vehicle_rel_spd_current_downstrm = double_value;
		}
		if ((index1 == 0) && (index2 == -1)) {
			Cur_VehData->vehicle_rel_spd_current_upstrm = double_value;
		}
		break;
	case DRIVER_DATA_VEH_LANE_ANGLE:
		Cur_VehData->current_veh_angle = double_value;
		Cur_VehData->current_veh_angle = double_value;
		break;
	case DRIVER_DATA_VEH_TYPE:
		Cur_VehData->vehicle_type = long_value;
		break;
	case DRIVER_DATA_NVEH_LENGTH:
		if ((index1 == 0) && (index2 == 1)) {
			Cur_VehData->vehicle_length = double_value;
		}

		/*For lane changing operations */
		if ((index1 == 1) && (index2 == 1)) {
			Cur_VehData->vehicle_length_left_downstrm = double_value;
		}
		if ((index1 == 1) && (index2 == -1)) {
			Cur_VehData->vehicle_length_left_upstrm = double_value;
		}
		if ((index1 == -1) && (index2 == 1)) {
			Cur_VehData->vehicle_length_right_downstrm = double_value;
		}
		if ((index1 == -1) && (index2 == -1)) {
			Cur_VehData->vehicle_length_right_upstrm = double_value;
		}
		break;
	}
}
Vehicle & Vehicle::VehicleNumber(long int Num)
{
	return *DataMap[Num];
}
SAE_VEHICLE::VehicleData & Vehicle::VehicleData(long int Num)
{
	return DataMap[Num]->Vehicle_Data;
}
double Vehicle::LateralDeviation()
{
	return Vehicle_Data.LateralDeviation();
}
void Vehicle::poll()
{
	/* This poll method allows for per-frame vehicle updates and tests*/

	if (Vehicle_Data.x_coordinate >= 10)
	{
		if (Vehicle_Data.Volume_set == false)
		{
			Vehicle::Simulation.Volume++;
			Vehicle_Data.Volume_set = true;
		}
	}

	if (Vehicle::Current_SAE() == SAE_5)
	{
		//In SAE5 over-ride control to always automated
		Control_Mode() = Automated_Control;
		return;
	}

	if (Vehicle_Data.x_coordinate > -6400
		&& Vehicle_Data.x_coordinate < -5600)
	{
		//For levels 0 -> 4 set human control for specific period
		Control_Mode() = Human_Control;
		return;
	}
	else if (Current_SAE() == SAE_4)
	{
		//In SAE level 4 outside of the above period, engage automated control
		Control_Mode() = Automated_Control;
		return;
	}

	if (Control_Mode() == Automated_Control && Vehicle_Data.current_velocity > 15.65)
	{
		//for SAE 0->3 Transfer control to human above the speed of 15.65
		Vehicle_Data.time_to_shift = Vehicle::Simulation.current_time + reaction_time();
		Control_Mode() = Human_Control;
	}

	if (Control_Mode() == Human_Control && Vehicle_Data.current_velocity < 15.65)
	{
		//for SAE 0->3 Transfer control to machine below the speed of 15.65
		Vehicle_Data.time_to_shift = Vehicle::Simulation.current_time + 2 * reaction_time();
		Control_Mode() = Automated_Control;
	}
}
void Vehicle::StoreVehicleFrame()
{
	static std::string data = "SAE" + std::to_string(SAE_LEVEL()) + "_" + SAE_TYPE() + "_";
	static std::string FilePathCombined = "SAE" + std::to_string(SAE_LEVEL()) + "_COMBINED_";
	static std::string filename = "C:\\DataLog\\" + data + '_' + CurTime() + ".csv";
	static std::ofstream Datafile(filename, std::ios_base::out);
	static std::string filename2 = "C:\\DataLog\\" + FilePathCombined + '_' + CurTime() + ".csv";
	static std::ofstream Datafile2(filename2, std::ios_base::out | std::ios_base::ate | std::ios_base::app);

	//The following code is used for debugging file-output if we "Cannot create or open" the file
	if (Datafile.fail())
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
	SS << Vehicle_Data.VehicleID << ','
		<< Vehicle_Data.vehicle_type << ','
		<< Vehicle_Data.Initial_link << ','
		<< Vehicle_Data.Initial_Lane << ','
		<< Vehicle_Data.final_link << ','
		<< CurTime() << ','
		<< Vehicle_Data.current_acceleration << ','
		<< Vehicle_Data.current_velocity << ','
		<< Vehicle_Data.relative_distance << ','
		<< Vehicle_Data.x_coordinate << ','
		<< Vehicle_Data.y_coordinate << ','
		<< Simulation.Volume << ','
		<< Vehicle_Data.lateral_position << ','
		<< Vehicle_Data.desired_lane_angle << ','
		<< Vehicle_Data.Random_value << ','
		<< Vehicle_Data.Change_volume
		<< '\n';
	//Take resulting stream and save to a single String.
	std::string output = SS.str();
	//output that string into our data-collection files 
	Datafile << output;
	Datafile2 << output;
	Datafile2.flush();
}
double Vehicle::DetermineLatPosValue()
{
	return Vehicle_Data.DetermineLatPosValue();
}
long Vehicle::DetermineLaneChangeValue()
{
	return Vehicle_Data.DetermineLaneChangeValue();
}


SAE_VEHICLE::VehicleData::VehicleData() : Initial_Lane(-1), Change_volume(0), Volume_set(false), final_link_set(false), Initial_link(-1), final_link(0), Lane_change_in_progress(false)
{


	//Variables declared here are defined in Simulation_Data.h
	cur_link = -1;
	cur_veh_lane = -1;
	number_of_lanes = -1;
	desired_acceleration = 0.0;
	desired_lane_angle = 0.0;
	active_lane_change = 0;
	rel_target_lane = 0;
	desired_velocity = 0.0;
	turning_indicator = 0;
	vehicle_color = RGB(0, 0, 0);
	current_velocity = 0.0;
	current_acceleration = 0.0;
	time_step = 0.0;

	jam_distance = 2.0;
	ratio = 0.0;
	desired_space_headway = 0.0;
	relative_velocity = 0.0;
	space_ratio = 0.0;
	space_headway = 0.0;
	relative_distance = 10000.0;
	vehicle_length = 0.0;
	vehicle_ID = 0.0;
	leading_veh_spd = 0.0;
	leading_veh_acc = 0.0;
	acc_idm = 0.0;
	effective_acc = 0.0;
	heaviside_step = 0;
	c = 0.99;
	b_safe = 4.0;
	time_ln_ch = 1.0;
	vehicle_ID_left_upstrm = 0;
	vehicle_ID_right_upstrm = 0;
	vehicle_ID_left_downstrm = 0;
	vehicle_ID_right_downstrm = 0;
	vehicle_ID_current_downstrm = 0;
	vehicle_ID_current_upstrm = 0;
	vehicle_ID_current_two_upstream = 0;
	vehicle_ID_current_two_downstream = 0;
	vehicle_rel_spd_left_upstrm = 0.0;
	vehicle_rel_spd_right_upstrm = 0.0;
	vehicle_rel_spd_left_downstrm = 0.0;
	vehicle_rel_spd_right_downstrm = 0.0;
	vehicle_rel_spd_current_downstrm = 0.0;
	vehicle_rel_spd_current_upstrm = 0.0;
	vehicle_headwy_left_upstrm = 0.0;
	vehicle_headwy_right_upstrm = 0.0;
	vehicle_headwy_left_downstrm = 0.0;
	vehicle_headwy_right_downstrm = 0.0;
	vehicle_headwy_current_upstrm = 0.0;
	vehicle_headwy_current_downstrm = 0.0;
	vehicle_length_left_upstrm = 0.0;
	vehicle_length_left_downstrm = 0.0;
	vehicle_length_right_upstrm = 0.0;
	vehicle_length_right_downstrm = 0.0;
	vehicle_length_right_self = 0.0;
	space_headway_left_upstream = 0.0;
	space_headway_left_self = 0.0;
	space_headway_right_upstream = 0.0;
	space_headway_right_self = 0.0;
	space_headway_current_upstream = 0.0;
	vehicle_acc_left_downstrm = 0.0;
	vehicle_acc_left_upstrm = 0.0;
	vehicle_acc_right_downstrm = 0.0;
	vehicle_acc_right_upstrm = 0.0;
	vehicle_acc_current_downstrm = 0.0;
	vehicle_acc_current_upstrm = 0.0;
	ratio_left_upstream = 0.0;
	ratio_left_downstream = 0.0;
	ratio_right_upstream = 0.0;
	ratio_right_downstream = 0.0;
	ratio_current_upstream = 0.0;
	vehicle_identity = 0;
	acc_idm_left_upstream = 0.0;
	acc_idm_right_upstream = 0.0;
	acc_idm_left_self = 0.0;
	acc_idm_right_self = 0.0;
	acc_idm_current_upstream = 0.0;
	acc_thr = .35;
	lane_change_to_left = 0.0;
	lane_change_to_right = 0.0;
	lateral_position = 0.0;
	current_veh_angle = 0.0;
	VehicleID = -1;
	vehicle_type = 0;
};

SAE_VEHICLE::StaticData::StaticData() :
	current_time(0), Volume(0)
{

}