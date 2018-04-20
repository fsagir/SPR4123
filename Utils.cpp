#include "Utils.h"
#include <chrono>

namespace utils {

	bool NormalDistribution::isHighwayDataInitialized = false;
	std::map<long, bool> NormalDistribution::highwayLinks;

	void NormalDistribution::initHighwayData() {
		highwayLinks[7] = true;
		isHighwayDataInitialized = true;
	}

	bool NormalDistribution::isVehicleOnHighway(VehicleData * vehicle) {
#if defined(SAE1_CAR) || defined(SAE2_CAR) || defined(SAE1_TRUCK) || defined(SAE2_TRUCK)
		return true;
#else
		while (!isHighwayDataInitialized)
			initHighwayData();
		return highwayLinks.find(vehicle->getCurLink()) != highwayLinks.end();
#endif

	}

	NormalDistribution::NormalDistribution(double mean, double variance) : m_mean(mean), m_variance(variance) {

	}

	double NormalDistribution::getNextDouble() {
		DistributionTruncationFrom_0_5_To_1_5 truncation(m_mean);
		return getTruncatedLogNormalDouble(m_mean, m_variance, truncation);
	}

	double NormalDistribution::getTruncatedLogNormalDouble(double mean, double variance, DistributionTruncation truncation) {
		double min_cutoff = truncation.getMinCutOff();
		double max_cutoff = truncation.getMaxCutOff();
		
		//std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
		std::default_random_engine generator(80);
		std::lognormal_distribution<double> distribution(mean, variance);
		double value = distribution(generator);
		while (true) {
			if (value >= min_cutoff && value <= max_cutoff)
				return value;
			value = distribution(generator);
		}
	}

	double NormalDistribution::getTruncatedNormalDouble(double mean, double variance, DistributionTruncation truncation) {
		double min_cutoff = truncation.getMinCutOff();
		double max_cutoff = truncation.getMaxCutOff();
		
		std::default_random_engine generator(std::chrono::system_clock::now().time_since_epoch().count());
		std::normal_distribution<double> distribution(mean, variance);
		double value = distribution(generator);
		while (true) {
			if (value >= min_cutoff && value <= max_cutoff)
				return value;
			value = distribution(generator);
		}
	}

	bool NormalDistribution::flipCoinWithProbability(double probability) {
		std::random_device rd;
		std::mt19937 engine(rd());
		std::bernoulli_distribution dist(probability);
		return dist(engine);
	}

	NormalDistribution::~NormalDistribution() {}

	DistributionTruncation::DistributionTruncation(double minCutOff, double maxCutOff) : minCutOff(minCutOff), maxCutOff(maxCutOff) { }
	
	DistributionTruncation::~DistributionTruncation() {}

	DistributionTruncation::DistributionTruncation(const DistributionTruncation & oldObj) {
		minCutOff = oldObj.minCutOff;
		maxCutOff = oldObj.maxCutOff;
	}
	
	double DistributionTruncation::getMinCutOff() {
		return minCutOff;
	}

	double DistributionTruncation::getMaxCutOff() {
		return maxCutOff;
	}


	DistributionTruncationFrom_0_5_To_1_5::DistributionTruncationFrom_0_5_To_1_5(double base) : DistributionTruncation(0.5 * base, 1.5 * base) { }
	DistributionTruncationFrom_0_5_To_1_5::~DistributionTruncationFrom_0_5_To_1_5() {}

	DistributionTruncationFrom_0_To_5::DistributionTruncationFrom_0_To_5() : DistributionTruncation(0, 5) {}
	DistributionTruncationFrom_0_To_5::~DistributionTruncationFrom_0_To_5() {}

}