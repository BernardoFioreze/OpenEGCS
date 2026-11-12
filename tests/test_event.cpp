#include <catch2/catch_test_macros.hpp>
#include <vector>
#include "Event.h"

using namespace openEGCS;

// `Event`'s comparison operators. Neither EventQueue (which orders the heap
// with its own comparator on (getTime(), seq), src/EventQueue.cpp) nor the
// end-to-end simulation calls them, so without this file they have no
// coverage at all. A recursive operator==(iterator) - `!(*this == it)`
// resolving to itself - would not fail these cases: it takes the whole test
// binary down with a stack overflow, which is the strongest form the
// regression test can take, since the defect is not a wrong answer but no
// answer.
//
// Deliberately NOT tested here: operator>/</>=/<= . They are inverted with
// respect to their own names (see Event.h). A test asserting what they
// currently do would lock the inversion in as intended behaviour, which is the
// opposite of useful.

namespace {

// Event(id, name, type, direction, destination, time, customer, elevator)
Event makeEvent(double time = 4.0, int floor = 3, int elevator = 1,
                unsigned int type = ENTER) {
	return Event(7, "ev", type, UP, floor, time, nullptr, elevator);
}

} // namespace

TEST_CASE("Event::operator==(iterator) returns an answer instead of recursing", "[event][regression]") {
	std::vector<Event> others{makeEvent()};
	Event self = makeEvent();

	// Pre-fix (`return !(*this == it)`) this single line never returned: it
	// resolved to this very operator and recursed until the stack ran out.
	REQUIRE(self == others.begin());
}

TEST_CASE("Event::operator==(iterator) separates equal from different events", "[event]") {
	Event self = makeEvent(/*time=*/4.0, /*floor=*/3, /*elevator=*/1);

	SECTION("equal by value") {
		std::vector<Event> v{makeEvent(4.0, 3, 1)};
		REQUIRE(self == v.begin());
		REQUIRE_FALSE(self != v.begin());
	}
	SECTION("different timestamp") {
		std::vector<Event> v{makeEvent(9.0, 3, 1)};
		REQUIRE_FALSE(self == v.begin());
		REQUIRE(self != v.begin());
	}
	SECTION("different floor") {
		std::vector<Event> v{makeEvent(4.0, 8, 1)};
		REQUIRE_FALSE(self == v.begin());
	}
	SECTION("different elevator") {
		std::vector<Event> v{makeEvent(4.0, 3, 2)};
		REQUIRE_FALSE(self == v.begin());
	}
	SECTION("different type") {
		std::vector<Event> v{makeEvent(4.0, 3, 1, LEAVE)};
		REQUIRE_FALSE(self == v.begin());
	}
}

// `==` and `!=` must be exact negations of each other. This is the property the
// pre-fix code violated most directly: `!=` compared the fields and `==` called
// itself, so the pair was not a pair at all.
TEST_CASE("Event's iterator == and != are exact negations", "[event]") {
	Event self = makeEvent(4.0, 3, 1);
	std::vector<Event> v{makeEvent(4.0, 3, 1), makeEvent(4.0, 8, 1), makeEvent(5.0, 3, 1)};

	for (auto it = v.begin(); it != v.end(); ++it) {
		REQUIRE((self == it) == !(self != it));
	}
}

// Documented contract of getStopId(): "ignored by operator==". Worth a case
// because the assignment operators *do* copy stopId, so the two halves of the
// class disagree on whether it is part of the value - on purpose.
TEST_CASE("Event's iterator == ignores stopId", "[event]") {
	Event self = makeEvent();
	self.setStopId(11);

	std::vector<Event> v{makeEvent()};
	v[0].setStopId(99);

	REQUIRE(self == v.begin());
}

// The two overloads describe the same relation and must not drift apart.
TEST_CASE("Event's iterator == agrees with the Event& overload", "[event]") {
	Event self = makeEvent(4.0, 3, 1);
	std::vector<Event> v{makeEvent(4.0, 3, 1), makeEvent(6.0, 3, 1)};

	for (auto it = v.begin(); it != v.end(); ++it) {
		Event copy = *it;
		REQUIRE((self == it) == (self == copy));
	}
}
