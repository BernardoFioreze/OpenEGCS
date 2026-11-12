#include "RandomProvider.h"

namespace openEGCS {

RandomProvider::RandomProvider(std::optional<uint64_t> seed)
	: engine(seed.has_value() ? *seed : std::random_device{}())
{
}

double RandomProvider::nextExponential(double rate)
{
	std::exponential_distribution<double> dist(rate);
	return dist(engine);
}

int RandomProvider::nextUniformInt(int minInclusive, int maxInclusive)
{
	std::uniform_int_distribution<int> dist(minInclusive, maxInclusive);
	return dist(engine);
}

double RandomProvider::nextNormal(double mean, double stddev)
{
	std::normal_distribution<double> dist(mean, stddev);
	return dist(engine);
}

} // namespace openEGCS
