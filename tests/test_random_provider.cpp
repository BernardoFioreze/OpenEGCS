#include <catch2/catch_test_macros.hpp>
#include "RandomProvider.h"

using namespace openEGCS;

TEST_CASE("RandomProvider with the same seed produces the same sequence", "[random]") {
	RandomProvider a(std::optional<uint64_t>(42));
	RandomProvider b(std::optional<uint64_t>(42));

	for (int i = 0; i < 20; i++) {
		REQUIRE(a.nextExponential(0.5) == b.nextExponential(0.5));
	}
}

TEST_CASE("RandomProvider with different seeds produces different sequences", "[random]") {
	RandomProvider a(std::optional<uint64_t>(1));
	RandomProvider b(std::optional<uint64_t>(2));

	bool anyDifferent = false;
	for (int i = 0; i < 20; i++) {
		if (a.nextExponential(0.5) != b.nextExponential(0.5)) {
			anyDifferent = true;
			break;
		}
	}
	REQUIRE(anyDifferent);
}

TEST_CASE("RandomProvider uniform int respects bounds", "[random]") {
	RandomProvider rng(std::optional<uint64_t>(7));
	for (int i = 0; i < 200; i++) {
		int v = rng.nextUniformInt(3, 8);
		REQUIRE(v >= 3);
		REQUIRE(v <= 8);
	}
}

TEST_CASE("RandomProvider normal with the same seed produces the same sequence", "[random]") {
	RandomProvider a(std::optional<uint64_t>(42));
	RandomProvider b(std::optional<uint64_t>(42));

	for (int i = 0; i < 20; i++) {
		REQUIRE(a.nextNormal(70.0, 15.0) == b.nextNormal(70.0, 15.0));
	}
}

TEST_CASE("RandomProvider normal samples are centered near the requested mean", "[random]") {
	RandomProvider rng(std::optional<uint64_t>(42));

	const double mean = 70.0;
	const double stddev = 15.0;
	const int n = 20000;
	double sum = 0;
	for (int i = 0; i < n; i++) {
		sum += rng.nextNormal(mean, stddev);
	}
	double sampleMean = sum / n;

	// Seed is fixed, so this is deterministic, not flaky. A generous
	// tolerance is used since the point is just to catch a broken
	// distribution (e.g. wrong mean/stddev wiring), not to validate the
	// standard library's normal_distribution implementation.
	REQUIRE(sampleMean > mean - 1.0);
	REQUIRE(sampleMean < mean + 1.0);
}
