/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : EventQueue.h

Copyright (C) 2011 Nivvedan, Ankush Das

    liftsim is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    liftsim is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with liftsim.  If not, see <http://www.gnu.org/licenses/>.


Authors:

Nivvedan
nivvedan@iitb.ac.in

Ankush Das
ankushdas@iitb.ac.in



File Created on : Wednesday, October 12, 2011.
Last modified on : Tuesday, 15 November, 2011.
Modified since 2026 by Bernardo Fioreze as part of OpenEGCS (see NOTICE).

*/

#ifndef __EVENTQUEUE__
#define __EVENTQUEUE__

#include<vector>
#include<functional>

#include "Event.h"

namespace openEGCS {

// Min-heap of Events ordered by (timestamp, insertion sequence), backed by
// std::vector + push_heap/pop_heap. Timestamp ties are broken by an explicit
// sequence number ("first inserted, first out"), which is what keeps the
// simulation deterministic byte for byte.
class EventQueue {

	private:
		struct Entry {
			Event event;
			unsigned long long seq;
			};

		std::vector<Entry> heap;
		unsigned long long nextSeq = 0;

		// True if `a` must be popped after `b`. Ordering is (time, seq),
		// both ascending. push_heap/pop_heap build a heap that is "max" with
		// respect to this comparator, so the entry that should pop out first
		// (earliest time, then earliest seq) is the one `later` ranks last.
		static bool later(const Entry& a, const Entry& b);

	public:
		Event pop();								// Precondition: !empty(). Throws std::logic_error if the queue is empty.
		int size() const;
		bool empty() const;

		const Event& push(Event& e);
		void increaseTime(int el_no, double inc_val, double lim);

		// Recomputes the timestamp of every pending event of elevator `el_no`.
		// `new_time(event)` returns the event's correct timestamp, or a negative
		// value to leave that event untouched (e.g. CUST_CALL and RECALL, which
		// are not tied to a route stop). Ties keep their original insertion order
		// (seq is not touched), so the result is deterministic. Callers hand over
		// the *current* truth, derived from the route, rather than a correction
		// relative to a previous guess. O(n); re-heapifies only if something changed.
		void retime(int el_no, const std::function<double(const Event&)>& new_time);

		// Is there a pending event of elevator `el_no` bound to route stop
		// `stopId` (Event::getStopId)?
		//
		// Used as "does this stop already have its ARRIVE": the test
		// ElevatorSystem::ensureArrivalEvent applies after every route insertion, so
		// that a task merged into an existing entry does not get a second arrival
		// for a stop that already has one. The queue is the right thing to ask
		// because it is the only place that knows what is actually still coming, so
		// no second register can disagree with it. O(n) in this elevator's pending
		// events.
		bool hasPendingForStop(int el_no, unsigned long int stopId) const;

		// Removes every pending event of elevator `el_no` that `pred` accepts
		// and returns them, earliest first (by the same (time, seq) order they
		// would have popped in, so the result is deterministic and does not
		// depend on the heap's internal layout). The heap is rebuilt only when
		// something was actually removed.
		//
		// This exists for load bypass and for stops withdrawn from a cabin: when a
		// stop leaves a route, its ARRIVE has to come out of the timeline with it.
		// It is the only operation that takes an event out of the queue other than
		// pop(), and the caller owns the consequence: a stop dropped without its
		// landing call being re-offered is a passenger dropped from the simulation,
		// which the Simulator's "every created customer reaches their destination"
		// check would catch, but only at the end of the run.
		std::vector<Event> extractPending(int el_no, const std::function<bool(const Event&)>& pred);

		};

} // namespace openEGCS

#endif
