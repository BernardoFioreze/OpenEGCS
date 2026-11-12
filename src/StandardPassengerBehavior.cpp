/*
StandardPassengerBehavior.cpp

See StandardPassengerBehavior.h.
*/

#include "StandardPassengerBehavior.h"

namespace openEGCS {

bool StandardPassengerBehavior::willBoard(const Customer& customer, const BoardingContext& cabin) const
{
	(void)customer;
	(void)cabin;
	return true;
}

double StandardPassengerBehavior::nextCallTime(const Customer& customer, const RefusalContext& refusal) const
{
	(void)customer;
	return refusal.cabinDeparture;
}

} // namespace openEGCS
