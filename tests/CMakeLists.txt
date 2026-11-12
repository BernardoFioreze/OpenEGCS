add_executable(liftsim_tests
	test_random_provider.cpp
	test_dispatch_strategy.cpp
	test_metrics_collector.cpp
	test_metrics_consistency.cpp
	test_engine_identities.cpp
	test_dwell_time.cpp
	test_dwell_presets.cpp
	test_door_hold.cpp
	test_route_order.cpp
	test_scheduling_injection.cpp
	test_fcfs_scheduling.cpp
	test_simulation_determinism.cpp
	test_json_snapshot.cpp
	test_snapshot_physics.cpp
	test_elevator_look.cpp
	test_elevator_position_bounds.cpp
	test_elevator_system.cpp
	test_stop_identity.cpp
	test_event_timestamps.cpp
	test_car_call_order.cpp
	test_dispatch_contract.cpp
	test_group_dispatch.cpp
	test_gahca_fitness.cpp
	test_beamurgia_estimator.cpp
	test_beamurgia_strategy.cpp
	test_peters_green.cpp
	test_step_mix_traffic.cpp
	test_gahca_strategy.cpp
	test_gahca_bench.cpp
	test_thv_dispatch.cpp
	test_load_bypass.cpp
	test_capacity_enforcement.cpp
	test_passenger_behavior.cpp
	test_traffic_profile.cpp
	test_down_peak_traffic.cpp
	test_up_lunch_traffic.cpp
	test_hall_call_model.cpp
	test_announced_direction.cpp
	test_stop_arrival.cpp
	test_event.cpp
	test_event_queue.cpp
	test_travel_time.cpp
	test_lift_kinematics.cpp
	test_lift_motor_model.cpp
	test_traffic_capacity.cpp
	test_run_config.cpp
)

target_link_libraries(liftsim_tests PRIVATE liftsim_core liftsim_examples Catch2::Catch2WithMain)

include(CTest)
include(Catch)
catch_discover_tests(liftsim_tests)

# The command line itself (src/main.cpp) has no Catch2 coverage - its parsing
# lives in main() - so these run the real binary. `<Stddev customer weight> 0`
# is a capacity in passengers (everybody weighs the mean) and must run to
# completion. A negative stddev must be refused by name, not substituted.
add_test(NAME cli_stddev_zero_is_a_capacity_in_passengers
	COMMAND liftsim 9 4 300 0.21 1 true 42 20 1 0 1.0)
add_test(NAME cli_stddev_negative_is_refused
	COMMAND liftsim 9 4 300 0.21 1 true 42 20 1 -1 1.0)
set_tests_properties(cli_stddev_negative_is_refused PROPERTIES
	PASS_REGULAR_EXPRESSION "Invalid Stddev customer weight")

# A value outside its valid range must be refused by name with exit code 1,
# never swapped for the argument's default in silence: a typo (a dropped sign,
# a misplaced decimal) would otherwise become a different scenario with no
# warning. One representative per kind: <= 0 (rate, mean weight), out of an
# interval (bypass fraction), and < 0 (a door time). The documented "<= 0
# means unset/unlimited" conventions (<Elevator max weight>, the
# dissatisfaction thresholds) are deliberately NOT refused and are not tested
# here.
add_test(NAME cli_zero_arrival_rate_is_refused
	COMMAND liftsim 9 4 300 0 1 true 42)
set_tests_properties(cli_zero_arrival_rate_is_refused PROPERTIES
	PASS_REGULAR_EXPRESSION "Invalid Arrival Rate")
add_test(NAME cli_zero_mean_weight_is_refused
	COMMAND liftsim 9 4 300 0.21 1 true 42 20 0)
set_tests_properties(cli_zero_mean_weight_is_refused PROPERTIES
	PASS_REGULAR_EXPRESSION "Invalid Mean customer weight")
add_test(NAME cli_bypass_fraction_above_one_is_refused
	COMMAND liftsim 9 4 300 0.21 1 true 42 20 70 15 1.5)
set_tests_properties(cli_bypass_fraction_above_one_is_refused PROPERTIES
	PASS_REGULAR_EXPRESSION "Invalid Load bypass threshold fraction")
add_test(NAME cli_negative_door_time_is_refused
	COMMAND liftsim 9 4 300 0.21 1 true 42 20 70 15 0.6 3.5 1000 1300 -1)
set_tests_properties(cli_negative_door_time_is_refused PROPERTIES
	PASS_REGULAR_EXPRESSION "Invalid Door open time")

# The CLI warns when the configured demand
# exceeds the estimated up-peak handling capacity. Both the firing and the
# silence are part of the contract, so both are tested - and only with a finite
# <Elevator max weight> (600 kg), because an unlimited cabin has no capacity to
# exceed. 3.0/s = 900 passengers/5 min is far above the ~169 these four cars can
# clear; 0.1/s = 30 is far below. Customer count does not affect the warning (it
# is a-priori), so it is kept small to stay fast.
add_test(NAME cli_saturation_warning_fires
	COMMAND liftsim 20 4 40 3.0 1 true 7 600 70 15)
set_tests_properties(cli_saturation_warning_fires PROPERTIES
	PASS_REGULAR_EXPRESSION "configured demand .* exceeds the estimated up-peak handling capacity")
add_test(NAME cli_no_saturation_warning_when_demand_fits
	COMMAND liftsim 20 4 40 0.1 1 true 7 600 70 15)
set_tests_properties(cli_no_saturation_warning_when_demand_fits PROPERTIES
	FAIL_REGULAR_EXPRESSION "configured demand .* exceeds")

# The JSON config front end. The byte-identity with
# the equivalent positional command is the correctness anchor (a config file is
# a nicer front end, not a different run), checked end to end by a script that
# runs both and diffs the snapshots. The unknown-key refusal is inline: a typo
# must fail loudly on stderr, the whole reason the format beats positional.
add_test(NAME cli_config_matches_positional
	COMMAND bash ${CMAKE_CURRENT_SOURCE_DIR}/config_byte_identical.sh
		$<TARGET_FILE:liftsim> ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/equivalent_to_positional.json)
add_test(NAME cli_config_unknown_key_is_refused
	COMMAND liftsim --config ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/unknown_key.json)
set_tests_properties(cli_config_unknown_key_is_refused PROPERTIES
	PASS_REGULAR_EXPRESSION "unknown key")
