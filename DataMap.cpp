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

double k = 0.0;

//Implemention of extern variable in Vehicle_data.h
std::map<int, VehicleData> DataMap; //This stores the vehicle data
extern std::map<int, long> vehicleLaneChangeMap;

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
		if ((x_coordinate > 9600) && (x_coordinate < 10900))
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
		if (DataMap[VehicleID].level_shift == Automated_Control && DataMap[VehicleID].current_velocity > 35)
		{
			DataMap[VehicleID].time_to_shift = DataMap[VehicleID].current_time + reaction_time;
			DataMap[VehicleID].level_shift = Human_Control;
		}
		//check to see if we are under human control and below 17.9 m/s speed limit
		if (DataMap[VehicleID].level_shift == Human_Control && DataMap[VehicleID].current_velocity < 35)
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
		if (vehicleLaneChangeMap.find(VehicleID) != vehicleLaneChangeMap.end()) {
			if (!DataMap[VehicleID].Lane_change_in_progress && vehicleLaneChangeMap[VehicleID] != 0) {
				DataMap[VehicleID].Change_volume = 1;
				DataMap[VehicleID].Lane_change_in_progress = true;
			}
			else if (DataMap[VehicleID].Lane_change_in_progress && vehicleLaneChangeMap[VehicleID] != 0) {
				DataMap[VehicleID].Change_volume = 0;
				DataMap[VehicleID].Lane_change_in_progress = true;
			}
			else {
				DataMap[VehicleID].Change_volume = 0;
				DataMap[VehicleID].Lane_change_in_progress = false;
			}
		}
		break;
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
		SS << DataPoint.current_time << ','
			<< DataPoint.VehicleID << ','
			<< DataPoint.cur_link << ','
			<< DataPoint.cur_veh_lane << ','
			<< DataPoint.x_coordinate << ','
			<< DataPoint.y_coordinate << ','
			<< DataPoint.x_coordinate_rear << ','
			<< DataPoint.y_coordinate_rear << ','
			<< DataPoint.length << ','
			<< DataPoint.width << ','
			<< DataPoint.current_velocity << ','
			<< DataPoint.current_acceleration << ','
			<< DataPoint.vehicle_type << ','
			<< DataPoint.Initial_link << ','
			<< DataPoint.Initial_Lane << ','
			<< DataPoint.Change_volume << ','
			<< DataPoint.Cur_veh_angle << ','
			<< DataPoint.lateral_position << ','

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
		<< DataMap[VehicleID].active_lane_change << ','
		<< active_lane_change << ','
		<<lateral_position<<','
		<< cur_link << ','
		<< cur_veh_lane << ','
		<< DataMap[VehicleID].level_shift<<','
		<< DataMap[VehicleID].time_to_shift<<','
		<<current_time<<','
		//<< pow(space_ratio,2) << ','
		//<< acc_acc << ','
		//<< acc_idm << ','
		//<< desired_space_headway << ','
		//<< relative_distance << ','
		//<< vehicle_ID << ','
		//<< DataMap[vehicle_ID].current_velocity << ','
		<< "\n";

	std::string output = SS.str();
	Datafile3 << output;
	Datafile3.flush();
}


//Implementation of constructor
#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
VehicleData::VehicleData() : a(0.0), b(0.0), curLaneCount(1), distanceFromSignal(-1), signalState(SIGNAL_STATE_OFF), curLink(0), curLane(0), isOddZoneTransitionActive(false), desired_velocity_initial(0), desired_velocity_final(0), desired_velocity_current(0), Initial_Lane(0), Change_volume(0), lane_set(false), Volume_set(false), link_set(false), final_link_set(false), Initial_link(0), final_link(0), Lane_change_in_progress(false), decided_to_stop_at_signal(false), decided_to_yeild(false)
#else
VehicleData::VehicleData() : a(0.0), b(0.0), curLaneCount(1), distanceFromSignal(-1), signalState(SIGNAL_STATE_OFF), curLink(0), curLane(0), Initial_Lane(0), Change_volume(0), lane_set(false), Volume_set(false), link_set(false), final_link_set(false), Initial_link(0), final_link(0), Lane_change_in_progress(false), decided_to_stop_at_signal(false), decided_to_yeild(false)
#endif
{
	y1 = 0;
	y2 = 0;
	y3 = 0;
	future_lateral_position = 0;
	level_shift = Automated_Control;
	time_to_shift = 0.0;
	Time_of_completion_of_lane_change = 0.0;
	Time_of_change_of_control_on_lane_angle = 0.0;
	deceleration_at_signal = 0.0;
	/*a = utils::NormalDistribution::getTruncatedLogNormalDouble(A_VD_MEAN, A_VD_VARIENCE, utils::DistributionTruncationFrom_0_To_5());
	b = utils::NormalDistribution::getTruncatedLogNormalDouble(B_VD_MEAN, B_VD_VARIENCE, utils::DistributionTruncationFrom_0_To_5());*/
	a = 1.4;
	b = 2;
	/*a = 1.5;*/
	deceleration_to_yeild = 0.0;
	x_coordinate_rear = 0.0;
	desired_velocity_previous_time_step = 0.0;
#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
	initOddZones();
#endif
}

VehicleData::VehicleData(const VehicleData & oldObj) {
	a = oldObj.a;
	b = oldObj.b;
	curLaneCount = oldObj.curLaneCount;
	distanceFromSignal = oldObj.distanceFromSignal;
	signalState = oldObj.signalState;
	curLink = oldObj.curLink;
	curLane = oldObj.curLane;
#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
	isOddZoneTransitionActive = oldObj.isOddZoneTransitionActive;
	desired_velocity_initial = oldObj.desired_velocity_initial;
	desired_velocity_final = oldObj.desired_velocity_final;
	desired_velocity_current = oldObj.desired_velocity_current;
	for (auto it = oldObj.oddZones.begin(); it != oldObj.oddZones.end(); it++) {
		oddZones.push_back(*it);
	}
#endif
	VehicleID = oldObj.VehicleID;
	lateral_position = oldObj.lateral_position;
	desired_lane_angle = oldObj.desired_lane_angle;
	Cur_veh_angle = oldObj.Cur_veh_angle;
	Random_value = oldObj.Random_value;
	current_time = oldObj.current_time;
	current_velocity = oldObj.current_velocity;
	current_acceleration = oldObj.current_acceleration;
	x_coordinate = oldObj.x_coordinate;
	y_coordinate = oldObj.y_coordinate;
	x_coordinate_rear = oldObj.x_coordinate_rear;
	relative_distance = oldObj.relative_distance;
	Initial_Lane = oldObj.Initial_Lane;
	vehicle_type = oldObj.vehicle_type;
	lane_set = oldObj.lane_set;
	final_link_set = oldObj.final_link_set;
	link_set = oldObj.link_set;
	Volume_set = oldObj.Volume_set;
	Initial_link = oldObj.Initial_link;
	final_link = oldObj.final_link;
	active_lane_change = oldObj.active_lane_change;
	Change_volume = oldObj.Change_volume;
	Lane_change_in_progress = oldObj.Lane_change_in_progress;
	y1 = oldObj.y1;
	y2 = oldObj.y2;
	y3 = oldObj.y3;
	future_lateral_position = oldObj.future_lateral_position;
	level_shift = oldObj.level_shift;
	time_to_shift = oldObj.time_to_shift;
	Time_of_completion_of_lane_change = oldObj.Time_of_completion_of_lane_change;
	Time_of_change_of_control_on_lane_angle = oldObj.Time_of_change_of_control_on_lane_angle;
	decided_to_stop_at_signal = oldObj.decided_to_stop_at_signal;
	decided_to_yeild = oldObj.decided_to_yeild;
	deceleration_at_signal = oldObj.deceleration_at_signal;
	deceleration_to_yeild = oldObj.deceleration_to_yeild;
	cur_link = oldObj.cur_link;
	cur_veh_lane = oldObj.cur_veh_lane;
	y_coordinate_rear = oldObj.y_coordinate_rear;
	length = oldObj.length;
	width = oldObj.width;
}

#if defined(SAE4_CAR) || defined(SAE4_TRUCK)
void VehicleData::initOddZones() {
	oddZones.push_back(std::pair<double, double>(0, 300));
}

void VehicleData::setDesiredVelocityIntial(double value) {
	desired_velocity_initial = value;
}

double VehicleData::getDesiredVelocityInitial() {
	return desired_velocity_initial;
}
void VehicleData::setDesiredVelocityFinal(double value) {
	desired_velocity_final = value;
}
double VehicleData::getDesiredVelocityFinal() {
	return desired_velocity_final;
}
void VehicleData::setDesiredVelocityStep(double value) {
	desired_velocity_current = value;
}
double VehicleData::getDesiredVelocityStep() {
	return desired_velocity_current;
}
double VehicleData::getNextDesiredVelocityStep(double velo_initial, double velo_final, double velo_current) {
	return velo_initial - ((velo_initial - velo_final) /
		(1 + exp(6 * velo_current - (velo_initial + velo_final) / 2)));
}

bool VehicleData::isTransitionOddZoneGoingOn() {
	return isOddZoneTransitionActive;
}

void VehicleData::setTransitionOddZone(bool value) {
	isOddZoneTransitionActive = value;
}

bool VehicleData::isCoordinateWithinOddZone(double x, double y) {
	for (auto it = oddZones.begin(); it != oddZones.end(); it++) {
		if (it->first <= x && x <= it->second)
			return true;
	}
	return false;
}
#endif


//std::map<long, std::map<long, std::vector<VehicleData>>> VehicleData::getVehicleDistanceFromSignal() {
//	std::map<long, std::map<long, std::vector<VehicleData>>> retValue;
//	for (auto it = DataMap.begin(); it != DataMap.end(); it++) {
//		if (it->second.getDistanceFromSignal() < 0) // no visible signal ahead so ignore it
//			continue;
//		retValue[it->second.getCurLink()][it->second.getCurLane()].push_back(it->second);
//	}
//
//	for (auto it = retValue.begin(); it != retValue.end(); it++) {
//		for (auto sit = it->second.begin(); sit != it->second.end(); sit++) {
//			std::sort(sit->second.begin(), sit->second.end(), [](VehicleData a, VehicleData b) {
//				return a.getDistanceFromSignal() < b.getDistanceFromSignal();
//			});
//		}
//	}
//
//	return retValue;
//}
//
//std::vector<VehicleData> VehicleData::getVehicleDistanceFromSignal(long linkNumber, long laneNumber) {
//	// call this method like VehicleData::getVehicleDistanceFromSignal(2, 3); 2nd link 3rd lane
//	// use for loop to loop throught the returned value
//	// for(int i=0; i < ret.size(); i++) {
//	//		ret[i].getDistanceFromSignal(); // 0 is the lowest distance, 1 is 2nd, 2 is third and so on
//	// }
//	auto retVal = VehicleData::getVehicleDistanceFromSignal();
//	if (retVal.find(linkNumber) == retVal.end())
//		return std::vector<VehicleData>(); // return empty array()
//	auto linkValues = retVal[linkNumber];
//	if (linkValues.find(laneNumber) == linkValues.end())
//		return std::vector<VehicleData>(); // return empty array()
//	return linkValues[laneNumber];
//}

void VehicleData::setCurLane(long curLane) {
	this->curLane = curLane;
}
long VehicleData::getCurLane() {
	return this->curLane;
}

void VehicleData::setCurLink(long curLink) {
	this->curLink = curLink;
}

long VehicleData::getCurLink() {
	return this->curLink;
}

double VehicleData::getConstA() {
	return a;
}

double VehicleData::getConstB() {
	return b;
}

void VehicleData::setLaneCount(int laneNumber) {
	this->curLaneCount = laneNumber;
}

int VehicleData::getLaneCount() {
	return this->curLaneCount;
}

bool VehicleData::isCurLaneLeftMost(long curLane) {
	return curLane == curLaneCount;
}

bool VehicleData::isCurLaneRightMost(long curLane) {
	return curLane == 1; // is the the rightmost lane == 1
}

void VehicleData::setDistanceFromSignal(double distance) {
	this->distanceFromSignal = distance;
}
double VehicleData::getDistanceFromSignal() {
	return this->distanceFromSignal;
}

void VehicleData::setSignalState(long state) {
	this->signalState = state;
}

long VehicleData::getSignalState() {
	return this->signalState;
}

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
	static const double Gamma1 = 2.26;
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
	if (lateral_position >= 1.5 || lateral_position <= -1.5 || future_lateral_position >= 1.5 || future_lateral_position <= -1.5)
		return 0;
	/*	future_lateral_position = abs((Beta1*W1 + Beta2*W2 + Beta3*W3 + RandomValue * I_t)*10/current_velocity);*/
	else
		return future_lateral_position;
	//return future_lateral_position;// / (current_velocity * 10000000000000000000);
	//return 0;
}