cmake_minimum_required(VERSION 3.16)
project(liftsim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE Release)
endif()

include(FetchContent)

FetchContent_Declare(
	json
	URL https://github.com/nlohmann/json/releases/download/v3.11.3/json.tar.xz
)
FetchContent_MakeAvailable(json)

FetchContent_Declare(
	Catch2
	GIT_REPOSITORY https://github.com/catchorg/Catch2.git
	GIT_TAG v3.5.4
)
FetchContent_MakeAvailable(Catch2)
list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")

# --- Core library -----------------------------------------------------------
# Public headers live in include/ (flat, namespace openEGCS). Implementation
# files live in src/. This is the "framework": entities (Customer, Elevator,
# Task, Event...), the event-driven engine (Simulator/ElevatorSystem/
# EventQueue) and the pluggable interfaces (IDispatchStrategy,
# IGroupDispatchStrategy, IElevatorSchedulingStrategy, IMetricsCollector,
# ITrafficProfile, ...) plus their default implementations. It intentionally
# does NOT bundle a default IDispatchStrategy - see examples/ for that.
set(LIFTSIM_CORE_SOURCES
	src/Customer.cpp
	src/CustomerFactory.cpp
	src/Elevator.cpp
	src/LookSchedulingStrategy.cpp
	src/LinearTravelTimeStrategy.cpp
	src/KinematicTravelTimeStrategy.cpp
	src/ElevatorSystem.cpp
	src/EventQueue.cpp
	src/Event.cpp
	src/Simulator.cpp
	src/Task.cpp
	src/Timer.cpp
	src/RandomProvider.cpp
	src/StandardMetricsCollector.cpp
	src/JsonSimulationLogger.cpp
	src/PercentageCapacityStrategy.cpp
	src/FixedDwellTimeStrategy.cpp
	src/DwellPresets.cpp
	src/DoorHoldDwellTimeStrategy.cpp
	src/StandardPassengerBehavior.cpp
	src/StationaryUniformTrafficProfile.cpp
	src/ImmediateGroupAdapter.cpp
	src/LiftKinematics.cpp
	src/LiftMotorModel.cpp
	src/TrafficCapacity.cpp
	src/RunConfig.cpp
)

add_library(liftsim_core STATIC ${LIFTSIM_CORE_SOURCES})
target_include_directories(liftsim_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(liftsim_core PUBLIC nlohmann_json::nlohmann_json)

# --- Example strategies -------------------------------------------------------
# Reference implementations of the extension interfaces (dispatch, group
# dispatch, route ordering, traffic profiles, passenger behaviour), kept
# separate from the core library on purpose: liftsim is a benchmarking
# platform, and each of these is *one* algorithm to compare custom ones
# against, never a hidden default.
add_library(liftsim_examples STATIC
	examples/greedy_nearest_dispatch/GreedyNearestDispatchStrategy.cpp
	examples/gahca_dispatch/GahcaFitness.cpp
	examples/gahca_dispatch/GahcaDispatchStrategy.cpp
	examples/thv_dispatch/ThvDispatchStrategy.cpp
	examples/traffic_profiles/PeakTrafficProfile.cpp
	examples/traffic_profiles/DownPeakTrafficProfile.cpp
	examples/traffic_profiles/UpPeakTrafficProfile.cpp
	examples/traffic_profiles/LunchPeakTrafficProfile.cpp
	examples/traffic_profiles/StepMixTrafficProfile.cpp
	examples/loading_threshold_behavior/LoadingThresholdPassengerBehavior.cpp
	examples/fcfs_scheduling/FcfsSchedulingStrategy.cpp
	examples/beamurgia_dispatch/BeamurgiaEstimator.cpp
	examples/beamurgia_dispatch/BeamurgiaDispatchStrategy.cpp
	examples/peters_green_dispatch/PetersGreenDispatchStrategy.cpp
)
target_include_directories(liftsim_examples PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/examples/greedy_nearest_dispatch
	${CMAKE_CURRENT_SOURCE_DIR}/examples/gahca_dispatch
	${CMAKE_CURRENT_SOURCE_DIR}/examples/thv_dispatch
	${CMAKE_CURRENT_SOURCE_DIR}/examples/traffic_profiles
	${CMAKE_CURRENT_SOURCE_DIR}/examples/loading_threshold_behavior
	${CMAKE_CURRENT_SOURCE_DIR}/examples/fcfs_scheduling
	${CMAKE_CURRENT_SOURCE_DIR}/examples/beamurgia_dispatch
	${CMAKE_CURRENT_SOURCE_DIR}/examples/peters_green_dispatch)
target_link_libraries(liftsim_examples PUBLIC liftsim_core)

# --- CLI executable ----------------------------------------------------------
add_executable(liftsim src/main.cpp)
target_link_libraries(liftsim PRIVATE liftsim_core liftsim_examples)

# --- Measurement tools (benches) ----------------------------------------------
# Not part of either library. Each runs one paper's scenario (or all the
# controllers under one environment), writes one snapshot per run, and has an
# analyze.py next to it that reads them; each source file's header says how.
#
# Beamurgia et al. 2016's scenarios under one GA variant.
add_executable(beamurgia_bench examples/beamurgia_dispatch/bench/BeamurgiaBench.cpp)
target_link_libraries(beamurgia_bench PRIVATE liftsim_core liftsim_examples)

# Cortes et al. 2004's saturated lunch-peak under GAHCA, THV or the greedy.
add_executable(gahca_bench examples/gahca_dispatch/bench/GahcaBench.cpp)
target_link_libraries(gahca_bench PRIVATE liftsim_core liftsim_examples)

# Peters 1998's lunch-time scenario with and without his Green Strategy No. 2
# (also writes runs.tsv).
add_executable(peters_bench examples/peters_green_dispatch/bench/PetersBench.cpp)
target_link_libraries(peters_bench PRIVATE liftsim_core liftsim_examples)

# Every shipped dispatch controller under one environment, the platform's
# defaults (also writes runs.tsv).
add_executable(dispatch_bench examples/dispatch_comparison/bench/DispatchBench.cpp)
target_link_libraries(dispatch_bench PRIVATE liftsim_core liftsim_examples)

enable_testing()
add_subdirectory(tests)
