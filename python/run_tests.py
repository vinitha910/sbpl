#!/usr/bin/env python
import os

def run_tests(num_tests):
	SBPL_MOBILITY_PKG_PATH = "/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility"

	test_num = 1
	while test_num <= num_tests:
		print "RUNNING HBSP TEST #" + str(test_num)
		scenario_name = "house/hbsp_test_" + str(test_num)
		name = "mobility_planner_hbsp_" + str(test_num)
		stats_filepath = SBPL_MOBILITY_PKG_PATH + "/scenarios/house/hbsp_stats.yaml"
		cmd = "roslaunch sbpl_mobility mobility_planner.launch scenario:=" + scenario_name + " name:=" + name + " stats_filepath:=" + stats_filepath
		print cmd
		test_num += 1
		#os.system(cmd)

	test_num = 1
	while test_num <= num_tests:
		print "RUNNING 2D DIJKSTRA TEST #" + str(test_num)
		scenario_name = "house/dijkstra_test_" + str(test_num)
		name = "mobility_planner_dijkstra_" + str(test_num)
		stats_filepath = SBPL_MOBILITY_PKG_PATH + "/scenarios/house/dijkstra_stats.yaml"
		cmd = "roslaunch sbpl_mobility mobility_planner.launch scenario:=" + scenario_name + " name:=" + name + " stats_filepath:=" + stats_filepath
		print cmd
		test_num += 1
		#os.system(cmd)

run_tests(61)