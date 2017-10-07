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
#include <cmath>
#include <random>
#include <chrono>

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

DRIVERMODEL_API  int  DriverModelSetValue(long   type,
	long   index1,
	long   index2,
	long   long_value,
	double double_value,
	char   *string_value)
{
	//Call dataSetValue (This function can be found in DataMap, defined in Vehicle_data.h)
	//	This effectively allows us to pass information from Vissim directly to our data collection
	//	upon completetion simulation code below runs normally.
	Vehicle::DataSetValue(type, index1, index2, long_value, double_value, string_value);


	return 1;
}



/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue(long   type,
	long   index1,
	long   index2,
	long   *long_value,
	double *double_value,
	char   **string_value)
{
	static Vehicle * & Cur_Vehicle = Vehicle::Simulation.Cur_Vehicle;
	static SAE_VEHICLE::VehicleData * & Cur_VehData = Vehicle::Simulation.Cur_VehData;
	/* Gets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, and writes that value to <*double_value>,     */
	/* <*float_value> or <**string_value> (object and value selection       */
	/* depending on <type>).                                                */
	/* Return value is 1 on success, otherwise 0.                           */

	switch (type) {
	case DRIVER_DATA_STATUS:
		*long_value = 0;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		*long_value = Cur_VehData->turning_indicator;
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		/* *double_value = current_velocity + current_acceleration * time_step; */
		*double_value = Cur_VehData->desired_velocity;
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		*long_value = Cur_VehData->vehicle_color;
		return 1;
	case DRIVER_DATA_WANTS_SUGGESTION:
		*long_value = 1;
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:

		*double_value = Cur_Vehicle->CalculateAccChange();
		return 1;
		//---------------------------------------------
		
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		/*RandomValue *= 100;*/
		if (Vehicle::Current_SAE() == SAE_0)
			*double_value = (Cur_Vehicle->LateralDeviation() - Cur_VehData->lateral_position) * 2 / Cur_VehData->current_velocity;
		else if (Vehicle::Current_SAE() == SAE_1)
			*double_value = (Cur_Vehicle->LateralDeviation() - Cur_VehData->lateral_position) * 10 / Cur_VehData->current_velocity;
		else
			*double_value = Cur_Vehicle->DetermineLatPosValue();

		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:
		if (Vehicle::Current_SAE() == SAE_0 || Vehicle::Current_SAE() == SAE_1 || Vehicle::Current_SAE() == SAE_2)
			*long_value = Cur_VehData->active_lane_change;
		else //SAE 3 -> 5
		{
			*long_value = Cur_Vehicle->DetermineLaneChangeValue();
		}

		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		*long_value = Cur_VehData->rel_target_lane;
		return 1;
	case DRIVER_DATA_SIMPLE_LANECHANGE:
			*long_value = 1;
		return 1;
	default:
		return 0;
	}
}

/*==========================================================================*/



/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/

