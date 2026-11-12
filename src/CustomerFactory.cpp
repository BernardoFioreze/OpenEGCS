#include <algorithm>
#include <stdexcept>
#include "CustomerFactory.h"
#include "StationaryUniformTrafficProfile.h"

namespace openEGCS {

CustomerFactory::CustomerFactory(std::shared_ptr<IRandomProvider> randomProvider, int max_floors, bool sane_customers,
	double mean_weight, double stddev_weight, std::shared_ptr<ITrafficProfile> traffic_profile)
	: rng(std::move(randomProvider)), maxFloors(max_floors), sane(sane_customers),
	  meanWeight(mean_weight), stddevWeight(stddev_weight),
	  // Null means the stationary uniform default - the rate it holds is
	  // irrelevant here, because only nextInterArrivalTime reads it and this
	  // factory never calls that method.
	  trafficProfile(traffic_profile ? std::move(traffic_profile)
	                                  : std::make_shared<StationaryUniformTrafficProfile>())
{
	if (stddevWeight < 0.0) {
		throw std::invalid_argument("CustomerFactory: stddev_weight must be >= 0 (0 = every customer weighs the mean)");
	}
}

Customer CustomerFactory::createArrival(unsigned long int id, double timestamp)
{
	// Origin/destination come from the traffic profile. The per-customer RNG
	// order is fixed: inter-arrival already drawn, then origin, then
	// destination (then the weight, below).
	int atFloor;
	int toFloor;
	trafficProfile->sampleTrip(timestamp, *rng, maxFloors, atFloor, toFloor);

	int direction;
	if (sane) {
		direction = (toFloor > atFloor) ? UP : DOWN;
	} else {
		direction = (toFloor > atFloor) ? DOWN : UP;
		if (atFloor == 0) direction = UP;
		if (atFloor == maxFloors) direction = DOWN;
	}

	// A Normal sample can land at/below zero, which makes no physical
	// sense for a weight. Clamp instead of re-drawing, so the RNG
	// consumption sequence stays fixed per customer (one draw each).
	//
	// A zero stddev keeps that one draw and discards it:
	// std::normal_distribution requires stddev > 0, so it is drawn with 1
	// instead, and skipping the draw altogether would make the floors and times
	// of a seed depend on whether the weights are spread.
	double draw = rng->nextNormal(meanWeight, stddevWeight > 0.0 ? stddevWeight : 1.0);
	double weight = std::max(1.0, stddevWeight > 0.0 ? draw : meanWeight);

	return Customer(id, "", timestamp, atFloor, toFloor, direction, weight);
}

} // namespace openEGCS
