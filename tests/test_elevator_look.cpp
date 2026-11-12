#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <stdexcept>
#include "Elevator.h"
#include "LinearTravelTimeStrategy.h"
#include <limits>
#include <memory>
#include "Task.h"
#include "Timer.h"

using namespace openEGCS;

// These tests document/guarantee the LOOK behaviour of Elevator::addTask:
// an elevator keeps sweeping in its current direction, serving every
// request found ahead of it, and only serves a request that requires a
// direction reversal once nothing is left ahead in the current sweep.
//
// NOTE on test technique: Elevator::processNextTask() only dequeues the
// current front task once the elevator's physical `position` matches that
// task's destination (that's what Elevator::update(Timer) advances). So to
// observe route order without depending on wall-clock simulation, each
// step below explicitly moves the elevator (one time unit per floor, so
// elapsed time == floors travelled) to the destination of the task currently
// at the front, then calls processNextTask() again to reveal the next one.
//
// Every elevator below therefore gets an explicit LinearTravelTimeStrategy(1).
// These cases are about ROUTE ORDER - that a LOOK cabin sweeps one way and
// then reverses - and the one-unit-per-floor clock is scaffolding that makes
// "move it to floor 7" expressible as "advance the clock by 7". Relying on
// Elevator's own fallback for it would make these cases break confusingly the
// day the fallback changes, for a reason having nothing to do with LOOK.

namespace {

// Peeks the task currently at the front of the queue (does not consume it).
Task peekFront(Elevator& e) {
	return e.processNextTask();
}

// Moves the elevator to `dest` (one time unit per floor in all tests below, so
// elapsed time == distance travelled) and pops the task that was at the
// front, returning whatever is now at the front of the queue.
Task advanceAndPop(Elevator& e, double& simTime, float dest) {
	simTime += std::abs(dest - e.getPosition());
	e.update(Timer(simTime));
	return e.processNextTask();
}

// One cabin on the linear clock, stated explicitly - see the note above.
Elevator lookElevator() {
	return Elevator(1, "", /*pos=*/0, /*time=*/1, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, nullptr,
		std::make_shared<LinearTravelTimeStrategy>(1.0));
}

} // namespace

TEST_CASE("Elevator starting IDLE picks direction from the first committed task", "[elevator][look]") {
	Elevator e = lookElevator();
	REQUIRE(e.getStatus() == IDLE);

	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1);

	REQUIRE(e.getStatus() == BUSY);
	REQUIRE(e.getDirection() == UP);
}

TEST_CASE("Elevator moving UP inserts a nearer request before a farther one", "[elevator][look]") {
	Elevator e = lookElevator();
	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1); // commits: direction = UP

	Task callAt3(0, "", CALL, UP, 3, 0);
	e.addTask(callAt3, 1);

	// route order must be [3, 5]: nearest-ahead-first while sweeping UP.
	REQUIRE(peekFront(e).getDest() == 3);
}

TEST_CASE("Elevator moving UP defers a DOWN hall call ahead of it until the reversal", "[elevator][look]") {
	Elevator e = lookElevator();
	double simTime = 0;

	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1); // direction = UP

	Task callAt3Up(0, "", CALL, UP, 3, 0);
	e.addTask(callAt3Up, 1);

	// A hall call requesting DOWN, even though its floor (4) is ahead of
	// the current position, must only be served after the current UP
	// sweep is exhausted (i.e. after floor 3 and floor 5).
	Task callAt4Down(0, "", CALL, DOWN, 4, 0);
	e.addTask(callAt4Down, 1);

	REQUIRE(peekFront(e).getDest() == 3);
	REQUIRE(advanceAndPop(e, simTime, 3).getDest() == 5);   // after serving 3, next is 5
	REQUIRE(advanceAndPop(e, simTime, 5).getDest() == 4);   // only after 5, the deferred DOWN call
}

TEST_CASE("Elevator moving UP orders deferred (behind/opposite) requests by proximity once reversed", "[elevator][look]") {
	Elevator e = lookElevator();
	double simTime = 0;

	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1); // direction = UP, position = 0

	// Both require a reversal: floor 2 (DOWN hall call, ahead of position
	// but wrong direction) and floor -1 (GO, behind position).
	Task callAt2Down(0, "", CALL, DOWN, 2, 0);
	e.addTask(callAt2Down, 1);

	Task goMinus1(0, "", GO, DEFAULT, -1, 0);
	e.addTask(goMinus1, 1);

	REQUIRE(peekFront(e).getDest() == 5);                    // 5 served first (current UP sweep)
	REQUIRE(advanceAndPop(e, simTime, 5).getDest() == 2);     // after reversal: nearer floor (2) first
	REQUIRE(advanceAndPop(e, simTime, 2).getDest() == -1);    // then the farther one (-1)
}

TEST_CASE("Elevator moving DOWN mirrors the UP behaviour", "[elevator][look]") {
	Elevator e(1, "", 10, 1);
	double simTime = 0;

	Task go2(0, "", GO, DEFAULT, 2, 0);
	e.addTask(go2, 1); // direction = DOWN
	REQUIRE(e.getDirection() == DOWN);

	Task callAt6Down(0, "", CALL, DOWN, 6, 0);
	e.addTask(callAt6Down, 1);

	// An UP hall call ahead (in the DOWN sweep sense) must be deferred.
	Task callAt4Up(0, "", CALL, UP, 4, 0);
	e.addTask(callAt4Up, 1);

	REQUIRE(peekFront(e).getDest() == 6);                     // nearest while going down: 6
	REQUIRE(advanceAndPop(e, simTime, 6).getDest() == 2);      // then 2 (still current sweep)
	REQUIRE(advanceAndPop(e, simTime, 2).getDest() == 4);      // only after reversal: deferred UP call at 4
}

TEST_CASE("Elevator addTask reuses an existing task instead of duplicating same destination", "[elevator][look]") {
	Elevator e = lookElevator();
	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1);

	Task go5Again(0, "", GO, DEFAULT, 5, 0);
	Task& result = e.addTask(go5Again, 1);
	REQUIRE(result.getDest() == 5);

	// Only one task should remain in the queue: after serving floor 5, the
	// elevator must go IDLE (the duplicate was not inserted).
	double simTime = 0;
	REQUIRE(peekFront(e).getDest() == 5);
	simTime += std::abs(5 - e.getPosition());
	e.update(Timer(simTime));
	e.processNextTask(); // erases the only task, queue becomes empty
	REQUIRE(e.getStatus() == IDLE);
}

TEST_CASE("Elevator addTask rejects an unknown task type instead of killing the process", "[elevator][look]") {
	Elevator e = lookElevator();
	Task go5(0, "", GO, DEFAULT, 5, 0);
	e.addTask(go5, 1); // status becomes BUSY, so the next addTask goes through the scheduling path

	Task unknown(0, "", DEFAULT, DEFAULT, 3, 0); // type == DEFAULT, neither GO nor CALL
	REQUIRE_THROWS_AS(e.addTask(unknown, 1), std::logic_error);
}

// A parked cabin handed a landing call at the floor it is standing at gets a
// leg of zero length, and must still be given a direction: the scheduling
// strategy is promised UP or DOWN (IElevatorSchedulingStrategy.h). With IDLE,
// LOOK would file the stop the cabin is standing at under "needs a reversal",
// the car call of whoever boarded there would go in front of it,
// processNextTask would find a different stop at the front and finish
// nothing, and the cabin would be stranded.
//
// Both directions are pinned: one of them comes out right by coincidence
// without the fix (the reversal group is sorted by floor ascending, and the
// stop is below the car call), which is the kind of thing that stops being
// true.
TEST_CASE("A parked cabin answering a landing call at its own floor takes the call's direction",
	"[elevator][look][regression]") {
	struct Case { unsigned int callDirection; float carCallTo; };
	const Case cases[] = { {DOWN, 2}, {UP, 9} };
	for (const Case& c : cases) {
		INFO((c.callDirection == UP ? "up-call" : "down-call") << " at 5, car call to " << c.carCallTo);
		Elevator e(1, "", /*pos=*/5, /*time=*/1, nullptr,
			std::numeric_limits<double>::infinity(), nullptr, nullptr,
			std::make_shared<LinearTravelTimeStrategy>(1.0));
		REQUIRE(e.getStatus() == IDLE);

		Task call(0, "", CALL, c.callDirection, 5, 0);
		Task placed = e.addTask(call, 1);
		REQUIRE(e.getStatus() == BUSY);
		REQUIRE(e.getDirection() == c.callDirection);		// was IDLE

		// Whoever boards registers their destination before the stop is finished,
		// and the stop the cabin is standing at must stay in front.
		Task carCall(0, "", GO, DEFAULT, c.carCallTo, 0);
		e.addTask(carCall, 1);
		REQUIRE(e.getRoute().front().getStopId() == placed.getStopId());

		// So finishing it by identity works, and the leg goes to the car call.
		REQUIRE(e.processNextTask(placed.getStopId()).getDest() == c.carCallTo);
		REQUIRE(e.getRoute().size() == 1);
	}
}

TEST_CASE("A car call to the floor a parked cabin is standing at is rejected, not given a direction",
	"[elevator][look][regression]") {
	// It has no direction of its own to lend the cabin, and nothing in the
	// engine produces one (CustomerFactory re-draws a destination equal to the
	// origin). Rather than invent a direction - which is what leaving IDLE did,
	// silently - the invariant "a BUSY cabin goes UP or DOWN" is checked.
	Elevator e(1, "", /*pos=*/5, /*time=*/1, nullptr,
		std::numeric_limits<double>::infinity(), nullptr, nullptr,
		std::make_shared<LinearTravelTimeStrategy>(1.0));
	Task goHere(0, "", GO, DEFAULT, 5, 0);
	REQUIRE_THROWS_AS(e.addTask(goHere, 1), std::logic_error);
}
