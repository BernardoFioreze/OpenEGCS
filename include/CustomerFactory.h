/*
CustomerFactory.h

Generates Customer instances using an injected IRandomProvider. This is
where the "which floor / which direction" randomness policy lives, kept
separate from the Customer entity itself so that:
  - Customer stays a plain data holder.
  - The randomness source (and therefore reproducibility via seed) is
    controlled from the outside (inversion of control).
*/

#ifndef __CUSTOMERFACTORY__
#define __CUSTOMERFACTORY__

#include <memory>
#include "Customer.h"
#include "IRandomProvider.h"
#include "ITrafficProfile.h"

namespace openEGCS {

class CustomerFactory {
	private:
		std::shared_ptr<IRandomProvider> rng;
		int maxFloors;
		bool sane;			// "Sane" customers pick a call direction consistent
							// with their destination floor; "insane" ones pick
							// the opposite (modelling irrational input).
		double meanWeight;		// Mean of the Normal(meanWeight, stddevWeight)
								// distribution customer weight is drawn from.
		double stddevWeight;	// >= 0. Zero means every customer weighs exactly
								// meanWeight (still clamped to >= 1, see createArrival).
		// Supplies the origin/destination floor pair. Never null: the constructor
		// falls back to a StationaryUniformTrafficProfile. The SAME instance is
		// shared with the Simulator (which uses its nextInterArrivalTime), so
		// demand stays one object.
		std::shared_ptr<ITrafficProfile> trafficProfile;

	public:
		// `stddev_weight` may be 0, which is how a capacity is expressed in
		// passengers rather than kilograms: mean 1, stddev 0 and a max weight of
		// 20 is a 20-person cabin. A zero stddev still consumes one Normal draw per
		// customer, so the arrival sequence (floors, directions, times) for a seed
		// does not depend on the weight spread - only the weights do. Throws
		// std::invalid_argument for a negative stddev, which describes no
		// distribution.
		//
		// `traffic_profile` supplies the origin/destination floor distribution.
		// A null profile means "the stationary uniform default". The Simulator
		// passes the same profile instance it uses for inter-arrival times; a test
		// constructing a factory alone can leave it null.
		CustomerFactory(std::shared_ptr<IRandomProvider> randomProvider, int max_floors, bool sane_customers = true,
			double mean_weight = 70.0, double stddev_weight = 15.0,
			std::shared_ptr<ITrafficProfile> traffic_profile = nullptr);

		// Creates a new Customer with a randomly generated origin/destination
		// floor pair (and a consistent call direction), arriving at `timestamp`.
		Customer createArrival(unsigned long int id, double timestamp);
};

} // namespace openEGCS

#endif
