/*

This file is a part of program liftsim for the course project of the course CS 293 - Data Structures and Algorithms Lab.

File Name : EventQueue.cpp

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


#include<algorithm>
#include<stdexcept>

#include "EventQueue.h"

namespace openEGCS {

bool EventQueue::later(const Entry& a, const Entry& b) {
	if (a.event.getTime() != b.event.getTime()) return a.event.getTime() > b.event.getTime();
	return a.seq > b.seq;
	}

int EventQueue::size() const {
	return static_cast<int>(heap.size());
	}

bool EventQueue::empty() const {
	return heap.empty();
	}

Event EventQueue::pop() {

	if (heap.empty()) throw std::logic_error("EventQueue::pop called on an empty queue");

	std::pop_heap(heap.begin(), heap.end(), later);
	Event result = std::move(heap.back().event);
	heap.pop_back();
	return result;
	}

const Event& EventQueue::push(Event& e) {

	heap.push_back(Entry{e, nextSeq++});
	std::push_heap(heap.begin(), heap.end(), later);
	return e;
	}

void EventQueue::increaseTime(int el_no, double inc_val, double lim)
{
	if (inc_val == 0) return;

	bool changed = false;
	for (Entry& entry : heap)
	{
		if ((entry.event.getElevator() == el_no) && (entry.event.getTime() > lim))
		{
			entry.event.increaseTime(inc_val);
			changed = true;
		}
	}
	if (changed) std::make_heap(heap.begin(), heap.end(), later);
}

void EventQueue::retime(int el_no, const std::function<double(const Event&)>& new_time)
{
	bool changed = false;
	for (Entry& entry : heap)
	{
		if (entry.event.getElevator() != el_no) continue;
		double t = new_time(entry.event);
		if (t < 0 || t == entry.event.getTime()) continue;
		entry.event.setTime(t);
		changed = true;
	}
	if (changed) std::make_heap(heap.begin(), heap.end(), later);
}

std::vector<Event> EventQueue::extractPending(int el_no,
	const std::function<bool(const Event&)>& pred)
{
	std::vector<Entry> kept;
	std::vector<Entry> removed;
	kept.reserve(heap.size());
	for (std::vector<Entry>::iterator it = heap.begin() ; it != heap.end() ; ++it)
	{
		if (it->event.getElevator() == el_no && pred(it->event)) removed.push_back(*it);
		else kept.push_back(*it);
	}
	std::vector<Event> result;
	if (removed.empty()) return result;

	heap.swap(kept);
	std::make_heap(heap.begin(), heap.end(), later);
	// Earliest first, by the queue's own order rather than by whatever order
	// the heap array happened to hold them in: the caller re-offers these to
	// the dispatcher, and that has to be reproducible for a seed.
	std::sort(removed.begin(), removed.end(), [](const Entry& a, const Entry& b) {
		return later(b, a);
	});
	for (std::vector<Entry>::iterator it = removed.begin() ; it != removed.end() ; ++it)
	{
		result.push_back(it->event);
	}
	return result;
}

bool EventQueue::hasPendingForStop(int el_no, unsigned long int stopId) const
{
	// stopId 0 means "not tied to a route stop" (CUST_CALL), and such an event
	// never gates a leg decision.
	if (stopId == 0) return false;
	for (std::vector<Entry>::const_iterator it = heap.begin() ; it != heap.end() ; ++it)
	{
		if (it->event.getElevator() != el_no) continue;
		if (it->event.getStopId() == stopId) return true;
	}
	return false;
}

} // namespace openEGCS
