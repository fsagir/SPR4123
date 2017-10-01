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

BOOL APIENTRY DllMain(HANDLE  hModule,
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
DRIVERMODEL_API  int  DriverModelGetValue(long   type,
	long   index1,
	long   index2,
	long   *long_value,
	double *double_value,
	char   **string_value)
{
	/* Gets the value of a data object of type <type>, selected by <index1> */
	/* and possibly <index2>, and writes that value to <*double_value>,     */
	/* <*float_value> or <**string_value> (object and value selection depending on <type>).       */
	/* Return value is 1 on success, otherwise 0.                           */



	switch (type) {
	case DRIVER_DATA_STATUS:
		*long_value = 0;
		return 1;
	case DRIVER_DATA_VEH_TURNING_INDICATOR:
		*long_value = turning_indicator;
		return 1;
	case DRIVER_DATA_VEH_DESIRED_VELOCITY:
		/* *double_value = current_velocity + current_acceleration * time_step; */
		*double_value = desired_velocity;
		return 1;
	case DRIVER_DATA_VEH_COLOR:
		*long_value = vehicle_color;
		return 1;
	case DRIVER_DATA_WANTS_SUGGESTION:
		*long_value = 1;
		return 1;
	case DRIVER_DATA_DESIRED_ACCELERATION:

		CalculateAccChange(double_value);

		return 1;
		//case DRIVER_DATA_DESIRED_LANE_ANGLE:
		//	/**double_value = desired_lane_angle;
		//	return 1;*/
	case DRIVER_DATA_DESIRED_LANE_ANGLE:
		/*RandomValue *= 100;*/
		DetermineLatPosValue(double_value);
		return 1;
	case DRIVER_DATA_ACTIVE_LANE_CHANGE:

		DetermineLaneChangeValue(long_value);
		return 1;
	case DRIVER_DATA_REL_TARGET_LANE:
		/**long_value = (rand() % 3) - 1;*/
		return 1;
	case DRIVER_DATA_SIMPLE_LANECHANGE:
		*long_value = 1;
		return 1;
	default:
		return 0;
	}
}

/*==========================================================================*/

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

/*==========================================================================*/
/*  Ende of DriverModel.cpp                                                 */
/*==========================================================================*/

