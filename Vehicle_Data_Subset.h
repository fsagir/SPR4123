#ifndef H_VEH_DATA_SUB_H
#define H_VEH_DATA_SUB_H
#include <cstdint>

enum Control_Mode 
{
	Human_Control = 1,
	Automated_Control = 0,
	SAE_ERROR
};
enum SAE_MODE_TYPE
{
	SAE_CAR, 
	SAE_TRUCK,
	SAE_ERROR_V
};
enum SAE_MODE
{
	SAE_0 =0,
	SAE_1 =1,
	SAE_2 =2,
	SAE_3 =3,
	SAE_4 =4,
	SAE_5 =5,
	SAE_ERROR_N
};
enum SAE_MODE_SPEC
{
	SAE_0_CAR, SAE_0_TRUCK,
	SAE_1_CAR, SAE_1_TRUCK,
	SAE_2_CAR, SAE_2_TRUCK,
	SAE_3_CAR, SAE_3_TRUCK,
	SAE_4_CAR, SAE_4_TRUCK,
	SAE_5_CAR, SAE_5_TRUCK,
	SAE_ERROR_S
};

class Vehicle;

namespace SAE_VEHICLE
{
	class VehicleData;

	class StaticData
	{
	public:
		StaticData();
		//current vehicle ID
		long Current_Vehicle_ID;
		static Vehicle * Cur_Vehicle;
		static SAE_VEHICLE::VehicleData * Cur_VehData;

		double current_time;
		uint32_t Volume;
		constexpr double a();
		constexpr double b();
		constexpr double p();
		constexpr double time_headway();
	};

	//structure to contain per-vehicle data from the simulation
	class VehicleData
	{
		friend class Vehicle;
	private:
		//structure to pass information to the IDM acceleration method
		struct ACC_idm_Data
		{
			ACC_idm_Data() = default;
			double veh_headwy;
			double veh_length;
			double veh_rel_spd;
			double current_vel;
			double desired_vel;
		};
		//calulate Acceleration values
		double	ACC_idm(ACC_idm_Data);
		double acc_idm;
		double Space_Ratio;

		//method to calulate automated lane changes
		long	CalculateAutomatedLaneChange(double & acc_idm);
		//method to calculate lateral deviation
		double LateralDeviation();
		//method to determine Acceleration value
		double DetermineAccValue(double acc_idm, double acc_acc);
	protected:
		//method to determine lane change selection
		long DetermineLaneChangeValue();
		//method to calulate acceleration change
		double	CalculateAccChange();
		//method to determine Lateral Position
		double DetermineLatPosValue();
	public:
		VehicleData();


		//Data provided with inherent getter methods
		long VehicleID;
		Control_Mode Current_Level;

		//Per Vehicle Data
		long vehicle_type;

		double x_coordinate;
		double y_coordinate;

		double relative_distance;
		double lateral_position;
		double desired_lane_angle;

		long Initial_Lane;
		long Initial_link;
		bool final_link_set;
		long final_link;

		uint32_t Change_volume;
		bool Lane_change_in_progress;

		double current_veh_angle;

		double  jam_distance;
		double  time_headway;
		double  time_ln_ch;

		double Random_value;
		double current_velocity;
		double current_acceleration;
		bool Volume_set;

	//TODO: not all of the following values are used - optimisation required

		double y1;
		double y2;
		double y3;
		double future_lateral_position;
		double time_to_shift;

		long	cur_link;
		long	cur_veh_lane;
		long	number_of_lanes;
		double  desired_acceleration;
		long    active_lane_change;
		long    rel_target_lane;
		double  desired_velocity;
		long    turning_indicator;
		long    vehicle_color;
		double  time_step;
		double  ratio;
		double  desired_space_headway;
		double  relative_velocity;
		double  space_ratio;
		double  space_headway;
		double  vehicle_length;
		double  vehicle_ID;
		double  leading_veh_spd;
		double  leading_veh_acc;

		double  effective_acc;
		long    heaviside_step;
		double  c;
		double  b_safe;
		int     vehicle_ID_left_upstrm;
		int     vehicle_ID_right_upstrm;
		int     vehicle_ID_left_downstrm;
		int     vehicle_ID_right_downstrm;
		int     vehicle_ID_current_downstrm;
		int     vehicle_ID_current_upstrm;
		int     vehicle_ID_current_two_upstream;
		int     vehicle_ID_current_two_downstream;
		double  vehicle_rel_spd_left_upstrm;
		double  vehicle_rel_spd_right_upstrm;
		double  vehicle_rel_spd_left_downstrm;
		double  vehicle_rel_spd_right_downstrm;
		double  vehicle_rel_spd_current_downstrm;
		double  vehicle_rel_spd_current_upstrm;
		double  vehicle_headwy_left_upstrm;
		double  vehicle_headwy_right_upstrm;
		double  vehicle_headwy_left_downstrm;
		double  vehicle_headwy_right_downstrm;
		double  vehicle_headwy_current_upstrm;
		double  vehicle_headwy_current_downstrm;
		double  vehicle_length_left_upstrm;
		double  vehicle_length_left_downstrm;
		double  vehicle_length_right_upstrm;
		double  vehicle_length_right_downstrm;
		double  vehicle_length_right_self;
		double  space_headway_left_upstream;
		double  space_headway_left_self;
		double  space_headway_right_upstream;
		double  space_headway_right_self;
		double  space_headway_current_upstream;
		double  vehicle_acc_left_downstrm;
		double	vehicle_acc_left_upstrm;
		double  vehicle_acc_right_downstrm;
		double	vehicle_acc_right_upstrm;
		double  vehicle_acc_current_downstrm;
		double	vehicle_acc_current_upstrm;
		double  ratio_left_upstream;
		double  ratio_left_downstream;
		double  ratio_right_upstream;
		double  ratio_right_downstream;
		double  ratio_current_upstream;
		long    vehicle_identity;
		double  acc_idm_left_upstream;
		double  acc_idm_right_upstream;
		double  acc_idm_left_self;
		double  acc_idm_right_self;
		double  acc_idm_current_upstream;
		double  acc_thr;
		double lane_change_to_left;
		double lane_change_to_right;
	};
};


#endif // H_VEH_DATA_SUB_H