#pragma once

#include <random>
#include <Vehicle_data.h>
#include <map>

//Default values depend on preprocessor Macro (set in project configuration)
#ifdef SAE0_CAR
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define LANE_CHANGE_PROBABILITY 0.8
#elif SAE1_CAR
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.01
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.01
#define LANE_CHANGE_PROBABILITY 0.8
#define REACTION_TIME_AUTO_2_HUMAN_MEAN 1.39 
#define REACTION_TIME_AUTO_2_HUMAN_VARIENCE 0.286
#define REACTION_TIME_HUMAN_2_AUTO_MEAN 1.45 
#define REACTION_TIME_HUMAN_2_AUTO_VARIENCE 0.324
#elif SAE2_CAR
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define LANE_CHANGE_PROBABILITY 0.8
#define REACTION_TIME_AUTO_2_HUMAN_MEAN 1.39 
#define REACTION_TIME_AUTO_2_HUMAN_VARIENCE 0.286
#define REACTION_TIME_HUMAN_2_AUTO_MEAN 1.45 
#define REACTION_TIME_HUMAN_2_AUTO_VARIENCE 0.324
#elif SAE3_CAR
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define REACTION_TIME_AUTO_2_HUMAN_MEAN 1.39 
#define REACTION_TIME_AUTO_2_HUMAN_VARIENCE 0.286
#define REACTION_TIME_HUMAN_2_AUTO_MEAN 1.45 
#define REACTION_TIME_HUMAN_2_AUTO_VARIENCE 0.324
#elif SAE4_CAR
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#elif SAE5_CAR
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#elif SAE0_TRUCK
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define LANE_CHANGE_PROBABILITY 0.8
#elif SAE1_TRUCK
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define LANE_CHANGE_PROBABILITY 0.8
#define REACTION_TIME_AUTO_2_HUMAN_MEAN 1.39 
#define REACTION_TIME_AUTO_2_HUMAN_VARIENCE 0.286
#define REACTION_TIME_HUMAN_2_AUTO_MEAN 1.45 
#define REACTION_TIME_HUMAN_2_AUTO_VARIENCE 0.324
#elif SAE2_TRUCK
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define LANE_CHANGE_PROBABILITY 0.8
#define REACTION_TIME_AUTO_2_HUMAN_MEAN 1.39 
#define REACTION_TIME_AUTO_2_HUMAN_VARIENCE 0.286
#define REACTION_TIME_HUMAN_2_AUTO_MEAN 1.45 
#define REACTION_TIME_HUMAN_2_AUTO_VARIENCE 0.324
#elif SAE3_TRUCK
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#define REACTION_TIME_AUTO_2_HUMAN_MEAN 1.39 
#define REACTION_TIME_AUTO_2_HUMAN_VARIENCE 0.286
#define REACTION_TIME_HUMAN_2_AUTO_MEAN 1.45 
#define REACTION_TIME_HUMAN_2_AUTO_VARIENCE 0.324
#elif SAE4_TRUCK
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#elif SAE5_TRUCK
#define A_VD_MEAN 0.31 
#define A_VD_VARIENCE 0.2
#define B_VD_MEAN 0.67
#define B_VD_VARIENCE 0.2
#else
#error SAE level not set
#endif // SAE0_CAR

namespace utils {

	class DistributionTruncation {
		double minCutOff;
		double maxCutOff;

	public:
		DistributionTruncation(double minCutOff, double maxCutOff);
		DistributionTruncation(const DistributionTruncation & oldObj);
		virtual ~DistributionTruncation();
		double getMinCutOff();
		double getMaxCutOff();
	};

	class DistributionTruncationFrom_0_5_To_1_5 : public DistributionTruncation {
	public:
		DistributionTruncationFrom_0_5_To_1_5(double base);
		virtual ~DistributionTruncationFrom_0_5_To_1_5();
	};

	class DistributionTruncationFrom_0_To_5 : public DistributionTruncation {
	public:
		DistributionTruncationFrom_0_To_5();
		virtual ~DistributionTruncationFrom_0_To_5();
	};


	class NormalDistribution {
		double m_mean;
		double m_variance;
		static bool isHighwayDataInitialized;
		static std::map<long, bool> highwayLinks;

		static void initHighwayData();

	public:
		NormalDistribution(double mean, double variance);
		virtual ~NormalDistribution();
		double getNextDouble();

		static double getTruncatedLogNormalDouble(double mean, double variance, DistributionTruncation truncation);
		static double getTruncatedNormalDouble(double mean, double variance, DistributionTruncation truncation);
		static bool flipCoinWithProbability(double probabiliy);
		static bool isVehicleOnHighway(VehicleData * vehicle);
	};
}