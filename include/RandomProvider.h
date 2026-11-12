/*
RandomProvider.h

Concrete IRandomProvider implementation based on <random>. Accepts an
optional seed: when a seed is supplied, the sequence of numbers generated
is fully deterministic (same seed => same sequence => same simulation),
which is what allows benchmarking different dispatch algorithms against
the exact same generated scenario. When no seed is supplied, a
non-deterministic seed source is used instead.
*/

#ifndef __RANDOMPROVIDER__
#define __RANDOMPROVIDER__

#include <random>
#include <optional>
#include <cstdint>

#include "IRandomProvider.h"

namespace openEGCS {

class RandomProvider : public IRandomProvider {
	private:
		std::mt19937_64 engine;

	public:
		explicit RandomProvider(std::optional<uint64_t> seed = std::nullopt);

		double nextExponential(double rate) override;
		int nextUniformInt(int minInclusive, int maxInclusive) override;
		double nextNormal(double mean, double stddev) override;
};

} // namespace openEGCS

#endif
