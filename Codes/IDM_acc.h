#pragma once


class Vehicle
{
public:
	int     vehicle_ID_left_upstrm = 0;
	int     vehicle_ID_right_upstrm = 0;
	int     vehicle_ID_left_downstrm = 0;
	int     vehicle_ID_right_downstrm = 0;
	int     vehicle_ID_current_downstrm = 0;
	int     vehicle_ID_current_upstrm = 0;

	//Constructor
	Vehicle(int V_id_LU, int V_id_RU);
	Vehicle();
private:

};

double ACC_idm(double veh_headwy, double veh_length, double veh_rel_spd,
	double time__ln_ch, double current_vel, double desired_vel, double jam__distance, double time__headway, double a_, double b_, double & space_ratio);