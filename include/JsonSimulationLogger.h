/*
JsonSimulationLogger.h

Default ISimulationLogger implementation. Buffers events in memory during
the run and serializes everything (timeline + metrics summary) as a
single JSON document on finalize(). Uses nlohmann::json (single header,
fetched via CMake FetchContent - see CMakeLists.txt).
*/

#ifndef __JSONSIMULATIONLOGGER__
#define __JSONSIMULATIONLOGGER__

#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include "ISimulationLogger.h"

namespace openEGCS {

struct LoggedEvent {
	double time;
	std::string type;
	int elevatorId;
	unsigned long int customerId;
	int floor;
};

struct LoggedCustomer {
	int atFloor;
	int toFloor;
	std::string direction;
	double arrivalTime;
};

class JsonSimulationLogger : public ISimulationLogger {
	private:
		std::vector<LoggedEvent> events;
		std::unordered_map<unsigned long int, LoggedCustomer> customers;
		std::optional<SimulationMetadata> metadata;

	public:
		void setMetadata(const SimulationMetadata& metadata) override;
		void logCustomer(const Customer& customer) override;
		void logEvent(double time, const std::string& type, int elevatorId, unsigned long int customerId, int floor) override;
		void finalize(const IMetricsCollector& metrics, const std::string& filePath) override;
};

} // namespace openEGCS

#endif
