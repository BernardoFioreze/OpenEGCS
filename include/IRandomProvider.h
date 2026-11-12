/*
IRandomProvider.h

Abstraction over randomness so the simulation can be run either with a
non-deterministic random source or with a fixed seed for fully
deterministic, reproducible runs (needed to compare different dispatch
algorithms on identical scenarios).
*/

#ifndef __IRANDOMPROVIDER__
#define __IRANDOMPROVIDER__

namespace openEGCS {

class IRandomProvider {
public:
	virtual ~IRandomProvider() = default;

	// Draws a sample from an Exponential(rate) distribution. Used to model
	// Poisson-process inter-arrival times of customers (rate = expected
	// number of arrivals per unit of time).
	virtual double nextExponential(double rate) = 0;

	// Draws a uniformly distributed integer in [minInclusive, maxInclusive].
	// Used e.g. to pick a random floor.
	virtual int nextUniformInt(int minInclusive, int maxInclusive) = 0;

	// Draws a sample from a Normal(mean, stddev) distribution. Used to
	// generate customer weight (see CustomerFactory).
	virtual double nextNormal(double mean, double stddev) = 0;
};

} // namespace openEGCS

#endif
