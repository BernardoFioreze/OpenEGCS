#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>
#include "Elevator.h"
#include "FixedDwellTimeStrategy.h"
#include "GreedyNearestDispatchStrategy.h"
#include "KinematicTravelTimeStrategy.h"
#include "LiftKinematics.h"
#include "PetersGreenDispatchStrategy.h"
#include "RandomProvider.h"
#include "Simulator.h"
#include "StandardMetricsCollector.h"
#include "StepMixTrafficProfile.h"
#include "Task.h"	// UP, DOWN, GO, CALL, DEFAULT

using namespace openEGCS;
using namespace openEGCS::examples;

// Peters' Green Strategy No. 2, a decorator around any dispatcher. Each
// clause of the condition has its own case.

namespace {

// Always answers the same cabin, so that the rule is the only thing that can
// change the answer.
class FixedChoice : public IDispatchStrategy {
	public:
		explicit FixedChoice(int id) : answer(id) {}
		int selectElevator(Task, std::vector<Elevator>&) override { return answer; }
	private:
		int answer;
};

Elevator cabinAt(unsigned long int id, double floor, double maxWeight = 1250.0)
{
	return Elevator(id, "", floor, 1, nullptr, maxWeight);
}

Elevator goingTo(unsigned long int id, double floor, int target, double maxWeight = 1250.0)
{
	Elevator e = cabinAt(id, floor, maxWeight);
	Task go(0, "", GO, DEFAULT, target, 0.0);
	e.addTask(go, 1);
	return e;
}

Task hallCall(int floor, unsigned int direction)
{
	return Task(0, "", CALL, direction, floor, 0.0);
}

PetersGreenDispatchStrategy green(int baseAnswer = 2)
{
	return PetersGreenDispatchStrategy(std::make_shared<FixedChoice>(baseAnswer));
}

} // namespace

TEST_CASE("The rule forces the cabin already stopping there and going that way", "[peters]") {
	std::vector<Elevator> cars{goingTo(1, 2.0, 8), cabinAt(2, 7.0)};
	PetersGreenDispatchStrategy s = green();
	REQUIRE(s.forcedCabin(hallCall(8, UP), cars) == 1);
	REQUIRE(s.selectElevator(hallCall(8, UP), cars) == 1);
	REQUIRE(s.forced() == 1);
}

TEST_CASE("The same stop going the other way does not qualify", "[peters]") {
	std::vector<Elevator> cars{goingTo(1, 2.0, 8), cabinAt(2, 7.0)};
	PetersGreenDispatchStrategy s = green();
	REQUIRE(s.forcedCabin(hallCall(8, DOWN), cars) == 0);
	REQUIRE(s.selectElevator(hallCall(8, DOWN), cars) == 2);
	REQUIRE(s.forced() == 0);
}

TEST_CASE("A cabin that will reach the floor only after reversing does not qualify", "[peters]") {
	// Going up from 12 with a car call at 15 and one at 8, behind it. A down-call
	// at 8 is ahead of it in the call's own direction, and it will stop there, but
	// it is not "travelling in the right direction" now.
	Elevator cabin = goingTo(1, 12.0, 15);
	Task back(0, "", GO, DEFAULT, 8, 0.0);
	cabin.addTask(back, 1);
	std::vector<Elevator> cars{cabin, cabinAt(2, 0.0)};
	REQUIRE(cabin.getDirection() == UP);
	REQUIRE(green().forcedCabin(hallCall(8, DOWN), cars) == 0);
}

TEST_CASE("A landing call on the route is not a car call", "[peters]") {
	Elevator cabin = cabinAt(1, 2.0);
	Task other = hallCall(8, UP);
	cabin.addTask(other, 1);
	std::vector<Elevator> cars{cabin, cabinAt(2, 7.0)};
	REQUIRE(green().forcedCabin(hallCall(8, UP), cars) == 0);
}

TEST_CASE("A floor already behind the cabin does not qualify", "[peters]") {
	// Going up from 5 with stops at 3 (behind) and 9: the stop at 3 comes on the
	// way back, in the other direction.
	Elevator cabin = goingTo(1, 5.0, 9);
	Task back(0, "", GO, DEFAULT, 3, 0.0);
	cabin.addTask(back, 1);
	std::vector<Elevator> cars{cabin, cabinAt(2, 0.0)};
	REQUIRE(green().forcedCabin(hallCall(3, UP), cars) == 0);
}

TEST_CASE("A parked cabin does not qualify, and the wrapped dispatcher decides", "[peters]") {
	std::vector<Elevator> cars{cabinAt(1, 4.0), cabinAt(2, 7.0)};
	PetersGreenDispatchStrategy s = green();
	REQUIRE(s.forcedCabin(hallCall(8, UP), cars) == 0);
	REQUIRE(s.selectElevator(hallCall(8, UP), cars) == 2);
}

TEST_CASE("A cabin that cannot answer does not qualify", "[peters]") {
	Elevator full = goingTo(1, 2.0, 8);
	Customer heavy(1, "", 0.0, 2, 8, UP, 1200.0);	// over the 60% bypass
	full.addCustomer(heavy);
	std::vector<Elevator> cars{full, cabinAt(2, 7.0)};
	REQUIRE(green().forcedCabin(hallCall(8, UP), cars) == 0);
}

TEST_CASE("Several qualify: the nearest, then the lowest id", "[peters]") {
	std::vector<Elevator> nearer{goingTo(1, 1.0, 8), goingTo(2, 5.0, 8), cabinAt(3, 7.0)};
	REQUIRE(green(3).forcedCabin(hallCall(8, UP), nearer) == 2);

	std::vector<Elevator> tied{goingTo(1, 5.0, 8), goingTo(2, 5.0, 8), cabinAt(3, 7.0)};
	REQUIRE(green(3).forcedCabin(hallCall(8, UP), tied) == 1);
}

TEST_CASE("The forced call adds no stop to the cabin", "[peters]") {
	// The point of the rule: the call merges into the stop the cabin already has.
	std::vector<Elevator> cars{goingTo(1, 2.0, 8), cabinAt(2, 7.0)};
	REQUIRE(cars[0].getRoute().size() == 1);
	Task call = hallCall(8, UP);
	cars[green().selectElevator(call, cars) - 1].addTask(call, 1);
	REQUIRE(cars[0].getRoute().size() == 1);
	REQUIRE(cars[0].getRoute().front().hasHallCall(UP));
	REQUIRE(cars[1].getRoute().empty());
}

TEST_CASE("selectElevator leaves the elevators as it found them", "[peters]") {
	std::vector<Elevator> cars{goingTo(1, 2.0, 8), goingTo(2, 3.0, 6)};
	green().selectElevator(hallCall(6, UP), cars);
	REQUIRE(cars[0].getRoute().size() == 1);
	REQUIRE(cars[1].getRoute().size() == 1);
	REQUIRE_FALSE(cars[1].getRoute().front().hasHallCall(UP));
}

TEST_CASE("The rule needs a dispatcher to wrap", "[peters]") {
	REQUIRE_THROWS_AS(PetersGreenDispatchStrategy(nullptr), std::invalid_argument);
}

namespace {

std::string runLunch(bool withRule, int cars, uint64_t seed, std::shared_ptr<PetersGreenDispatchStrategy>* out = nullptr)
{
	const int floors = 13;
	const double height = 3.6;
	const LiftDynamics dynamics{1.875, 0.375, 0.375};
	StepMixTrafficProfile::Params traffic;
	traffic.populatedFloors = floors;
	traffic.populationPerFloor = 80.0;
	traffic.outgoing = 0.3;
	traffic.interfloor = 0.7;
	traffic.minPercent = 6.0;
	traffic.maxPercent = 6.0;

	std::shared_ptr<IDispatchStrategy> dispatch = std::make_shared<GreedyNearestDispatchStrategy>();
	if (withRule) {
		auto rule = std::make_shared<PetersGreenDispatchStrategy>(dispatch);
		if (out) *out = rule;
		dispatch = rule;
	}
	std::streambuf* saved = std::cout.rdbuf();
	std::ostringstream sink;
	std::cout.rdbuf(sink.rdbuf());
	Simulator sim(seed, "peters", floors, cars, 200, 0.5, 1.0f, true,
		std::make_shared<RandomProvider>(std::optional<uint64_t>(seed)), dispatch,
		std::make_shared<StandardMetricsCollector>(1.0, height, dynamics), nullptr,
		std::optional<uint64_t>(seed), 1250.0, nullptr, 75.0, 15.0,
		std::make_shared<FixedDwellTimeStrategy>(1.8, 2.9, 1.2),
		std::make_shared<KinematicTravelTimeStrategy>(height, dynamics), nullptr, nullptr,
		std::make_shared<StepMixTrafficProfile>(traffic));
	sim.startSimulation();
	const std::string path = "peters_test_snapshot.json";
	sim.writeSnapshot(path);
	std::cout.rdbuf(saved);
	std::ifstream in(path);
	std::stringstream contents;
	contents << in.rdbuf();
	std::remove(path.c_str());
	return contents.str();
}

size_t count(const std::string& text, const std::string& word)
{
	size_t n = 0, at = 0;
	while ((at = text.find(word, at)) != std::string::npos) { ++n; ++at; }
	return n;
}

} // namespace

TEST_CASE("A full run: everybody arrives, and the same seed gives the same snapshot", "[peters]") {
	std::shared_ptr<PetersGreenDispatchStrategy> rule;
	const std::string first = runLunch(true, 6, 5, &rule);
	REQUIRE(first == runLunch(true, 6, 5));
	REQUIRE(count(first, "\"LEAVE\"") == 200);
	REQUIRE(rule->decisions() > 0);
}

TEST_CASE("With one cabin the rule has nothing to overrule, and the run does not change", "[peters]") {
	// Nothing in the decorator may move a run on its own: it draws nothing, and
	// asking the wrapped dispatcher is the same call the plain run makes.
	std::shared_ptr<PetersGreenDispatchStrategy> rule;
	const std::string with = runLunch(true, 1, 5, &rule);
	REQUIRE(rule->forced() == 0);
	REQUIRE(with == runLunch(false, 1, 5));
}
