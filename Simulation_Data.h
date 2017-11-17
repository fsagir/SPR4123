#pragma once
// Include the header file for our generic driver-model (provided by VISIM)
#include "DriverModel.h"
// Include the header file for simulation logging / Data output
#include "Vehicle_data.h"
//Standard C++ Header files for mathematics and dynamic contigious arrays.
#include <cmath>
#include <vector>
// declare variables for use in the simulation
//	Note: variables are declared as Extern to prevent duplicate symbol errors
//		ie: only one copy of each variable exists, no matter how many times this 
//		header is included.
extern long		cur_link;
extern long	   cur_veh_lane;
extern long	 number_of_lanes;
extern double  desired_acceleration ;
extern double  desired_lane_angle ;
extern long    active_lane_change ;
extern long    rel_target_lane ;
extern double  desired_velocity ;
extern long    turning_indicator ;
extern long    vehicle_color ;
extern double  current_velocity ;
extern double  current_acceleration ;
extern double  time_step ;
extern double  a ;
extern double  b ;
extern double  jam_distance ;
extern double  time_headway ;
extern double  ratio ;
extern double  desired_space_headway ;
extern double  relative_velocity ;
extern double  space_ratio ;
extern double  space_headway ;
extern double  relative_distance ;
extern double  vehicle_length ;
extern double  vehicle_ID ;
extern double  leading_veh_spd ;
extern double  leading_veh_acc ;
extern double  acc_idm ;
extern double  acc_cah ;
extern double  acc_acc ;
extern double  effective_acc ;
extern long    heaviside_step ;
extern double  c ;
extern double  b_safe ;
extern double  time_ln_ch ;
extern int     vehicle_ID_left_upstrm ;
extern int     vehicle_ID_right_upstrm ;
extern int     vehicle_ID_left_downstrm ;
extern int     vehicle_ID_right_downstrm ;
extern int     vehicle_ID_current_downstrm ;
extern int     vehicle_ID_current_upstrm ;
extern int     vehicle_ID_current_two_upstream;
extern int     vehicle_ID_current_two_downstream;
extern double  vehicle_rel_spd_left_upstrm ;
extern double  vehicle_rel_spd_right_upstrm ;
extern double  vehicle_rel_spd_left_downstrm ;
extern double  vehicle_rel_spd_right_downstrm ;
extern double  vehicle_rel_spd_current_downstrm ;
extern double  vehicle_rel_spd_current_upstrm ;
extern double  vehicle_headwy_left_upstrm ;
extern double  vehicle_headwy_right_upstrm ;
extern double  vehicle_headwy_left_downstrm ;
extern double  vehicle_headwy_right_downstrm ;
extern double  vehicle_headwy_current_upstrm ;
extern double  vehicle_headwy_current_downstrm ;
extern double  vehicle_length_left_upstrm ;
extern double  vehicle_length_left_downstrm ;
extern double  vehicle_length_right_upstrm ;
extern double  vehicle_length_right_downstrm ;
extern double  vehicle_length_right_self ;
extern double  space_headway_left_upstream ;
extern double  space_headway_left_self ;
extern double  space_headway_right_upstream ;
extern double  space_headway_right_self ;
extern double  space_headway_current_upstream ;
extern double  vehicle_acc_left_downstrm ;
extern double	vehicle_acc_left_upstrm ;
extern double  vehicle_acc_right_downstrm ;
extern double	vehicle_acc_right_upstrm ;
extern double  vehicle_acc_current_downstrm ;
extern double	vehicle_acc_current_upstrm ;
extern double  ratio_left_upstream ;
extern double  ratio_left_downstream ;
extern double  ratio_right_upstream ;
extern double  ratio_right_downstream ;
extern double  ratio_current_upstream ;
extern std::vector<double>    vehicle_desired_vel_array;
extern std::vector<double>    vehicle_X_coordinate_array;
extern long    vehicle_identity ;
extern double  acc_idm_left_upstream ;
extern double  acc_idm_right_upstream ;
extern double  acc_idm_left_self ;
extern double  acc_idm_right_self ;
extern double  acc_idm_current_upstream ;
extern double  p ;
extern double  acc_thr ;
extern double lane_change_to_left ;
extern double lane_change_to_right ;
extern double current_time ;
extern double lateral_position;
extern double desired_lane_angle;
extern double current_veh_angle;
extern long VehicleID;
extern double x_coordinate;
extern double y_coordinate;
extern bool level_shift;
extern double time_to_shift;
extern double reaction_time;
extern double lateral_position;
extern double desired_angle;
//extern bool lane_change_in_progress;
//extern bool completion_of_lane_change;
//extern double change_of_control_on_angle_time;
extern double duration_of_vissim_conrol_on_angle_after_lane_change;
extern long MOBIL_active_lane_change;
extern long lane_change_for_SAE_level;




void DetrimeAccValue(double * double_value);
void DetermineLatPosValue(double * double_value);
void DetermineLaneChangeValue(long * long_value);


void CalculateAutomatedLaneChange(long* long_value);

void CalculateAccChange(double * double_value);