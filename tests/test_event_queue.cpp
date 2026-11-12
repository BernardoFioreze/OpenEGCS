#include <catch2/catch_test_macros.hpp>
#include <random>
#include <vector>
#include <list>
#include "EventQueue.h"
#include "Event.h"

using namespace openEGCS;

// Characterization tests: they pin down the observable behavior of the queue
// (ordering, tie-breaking, increaseTime semantics), which the simulation's
// determinism depends on.

TEST_CASE("EventQueue::pop returns events in increasing time order", "[event_queue]") {
	EventQueue q;
	Event e1(1, "", CUST_CALL, DEFAULT, 0, 5.0, nullptr, 1);
	Event e2(2, "", CUST_CALL, DEFAULT, 0, 1.0, nullptr, 1);
	Event e3(3, "", CUST_CALL, DEFAULT, 0, 3.0, nullptr, 1);
	q.push(e1);
	q.push(e2);
	q.push(e3);

	REQUIRE(q.pop().getId() == 2);
	REQUIRE(q.pop().getId() == 3);
	REQUIRE(q.pop().getId() == 1);
	REQUIRE(q.empty());
}

TEST_CASE("EventQueue breaks ties by insertion order", "[event_queue]") {
	EventQueue q;
	Event e1(1, "", CUST_CALL, DEFAULT, 0, 2.0, nullptr, 1);
	Event e2(2, "", CUST_CALL, DEFAULT, 0, 2.0, nullptr, 1);
	Event e3(3, "", CUST_CALL, DEFAULT, 0, 2.0, nullptr, 1);
	q.push(e1);
	q.push(e2);
	q.push(e3);

	REQUIRE(q.pop().getId() == 1);
	REQUIRE(q.pop().getId() == 2);
	REQUIRE(q.pop().getId() == 3);
}

TEST_CASE("EventQueue::increaseTime only touches events of the given elevator past the limit", "[event_queue]") {
	EventQueue q;
	Event eA(1, "", CUST_CALL, DEFAULT, 0, 10.0, nullptr, 1); // elevator 1, time > lim -> touched
	Event eB(2, "", CUST_CALL, DEFAULT, 0, 10.0, nullptr, 2); // elevator 2 -> untouched
	Event eC(3, "", CUST_CALL, DEFAULT, 0, 5.0, nullptr, 1);  // elevator 1, time == lim (not >) -> untouched
	Event eD(4, "", CUST_CALL, DEFAULT, 0, 4.0, nullptr, 1);  // elevator 1, time < lim -> untouched
	q.push(eA);
	q.push(eB);
	q.push(eC);
	q.push(eD);

	q.increaseTime(1, 100.0, 5.0);

	// Pop everything in time order and check which timestamps moved.
	std::vector<std::pair<unsigned long int, double>> seen;
	while (!q.empty()) {
		Event e = q.pop();
		seen.push_back({e.getId(), e.getTime()});
	}

	REQUIRE(seen.size() == 4);
	for (auto& [id, time] : seen) {
		if (id == 1) REQUIRE(time == 110.0);
		else if (id == 2) REQUIRE(time == 10.0);
		else if (id == 3) REQUIRE(time == 5.0);
		else if (id == 4) REQUIRE(time == 4.0);
	}
}

TEST_CASE("EventQueue::increaseTime with inc == 0 changes nothing", "[event_queue]") {
	EventQueue q;
	Event eA(1, "", CUST_CALL, DEFAULT, 0, 10.0, nullptr, 1);
	Event eB(2, "", CUST_CALL, DEFAULT, 0, 20.0, nullptr, 1);
	q.push(eA);
	q.push(eB);

	q.increaseTime(1, 0.0, 0.0);

	REQUIRE(q.pop().getTime() == 10.0);
	REQUIRE(q.pop().getTime() == 20.0);
}

namespace {

// Minimal re-implementation of the pre-refactor algorithm (list + linear
// scan + strict `<`), embedded here as the reference model for the
// differential test (Task 4). Intentionally independent of EventQueue's
// current implementation.
class ReferenceEventQueue {
	public:
		void push(const Event& e) { theQueue.push_back(e); }

		Event pop() {
			auto min_it = theQueue.begin();
			double min_time = min_it->getTime();
			for (auto it = theQueue.begin(); it != theQueue.end(); ++it) {
				if (it->getTime() < min_time) {
					min_time = it->getTime();
					min_it = it;
				}
			}
			Event saved = *min_it;
			theQueue.erase(min_it);
			return saved;
		}

		void increaseTime(int el_no, double inc_val, double lim) {
			for (auto& e : theQueue) {
				if (e.getElevator() == el_no && e.getTime() > lim) {
					e.increaseTime(inc_val);
				}
			}
		}

		bool empty() const { return theQueue.empty(); }
		size_t size() const { return theQueue.size(); }

	private:
		std::list<Event> theQueue;
};

} // namespace

TEST_CASE("EventQueue matches the pre-refactor reference model under random operations", "[event_queue][differential]") {
	std::mt19937 rng(12345);
	// Small integer time pool to force lots of ties.
	std::uniform_int_distribution<int> timeDist(0, 9);
	std::uniform_int_distribution<int> elevatorDist(1, 3);
	std::uniform_int_distribution<int> opDist(0, 9);

	EventQueue actual;
	ReferenceEventQueue expected;
	unsigned long int nextId = 1;
	size_t liveCount = 0;

	for (int i = 0; i < 5000; ++i) {
		int op = opDist(rng);
		if (op < 6 || liveCount == 0) {
			// push
			unsigned long int id = nextId++;
			int elevator = elevatorDist(rng);
			double time = static_cast<double>(timeDist(rng));
			Event e(id, "", CUST_CALL, DEFAULT, 0, time, nullptr, elevator);
			actual.push(e);
			expected.push(e);
			++liveCount;
		} else if (op < 9) {
			// pop
			Event a = actual.pop();
			Event b = expected.pop();
			REQUIRE(a.getId() == b.getId());
			REQUIRE(a.getTime() == b.getTime());
			REQUIRE(a.getElevator() == b.getElevator());
			REQUIRE(a.getType() == b.getType());
			--liveCount;
		} else {
			// increaseTime
			int elevator = elevatorDist(rng);
			double inc = static_cast<double>(timeDist(rng));
			double lim = static_cast<double>(timeDist(rng));
			actual.increaseTime(elevator, inc, lim);
			expected.increaseTime(elevator, inc, lim);
		}
	}

	while (liveCount > 0) {
		Event a = actual.pop();
		Event b = expected.pop();
		REQUIRE(a.getId() == b.getId());
		REQUIRE(a.getTime() == b.getTime());
		--liveCount;
	}
	REQUIRE(actual.empty());
	REQUIRE(expected.empty());
}
