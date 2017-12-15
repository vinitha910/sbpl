import os

def run_tests(num_tests):
	SBPL_MOBILITY_PKG_PATH = "/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility"

	test_num = 1
	while test_num <= num_tests:
		print "RUNNING HBSP TEST #" + str(test_num)
		scenario_name = "house/hbsp_test_" + str(test_num)
		name = "mobility_planner_hbsp_" + str(test_num)
		stats_filepath = SBPL_MOBILITY_PKG_PATH + "/scenarios/house/hbsp_stats.yaml"
		
		dir_name = SBPL_MOBILITY_PKG_PATH + "/scenarios/" + scenario_name
		if os.path.exists(dir_name):
			cmd = "roslaunch sbpl_mobility mobility_planner.launch scenario:=" + scenario_name + " name:=" + name + " stats_filepath:=" + stats_filepath + " test_num:=" + str(test_num)
			print cmd
			os.system(cmd)
		test_num += 1

	test_num = 1
	while test_num <= num_tests:
		print "RUNNING 2D DIJKSTRA TEST #" + str(test_num)
		scenario_name = "house/dijkstra_test_" + str(test_num)
		name = "mobility_planner_dijkstra_" + str(test_num)
		stats_filepath = SBPL_MOBILITY_PKG_PATH + "/scenarios/house/dijkstra_stats.yaml"
		
		dir_name = SBPL_MOBILITY_PKG_PATH + "/scenarios/" + scenario_name
		if os.path.exists(dir_name):
			cmd = "roslaunch sbpl_mobility mobility_planner.launch scenario:=" + scenario_name + " name:=" + name + " stats_filepath:=" + stats_filepath + " test_num:=" + str(test_num)
			print cmd
			os.system(cmd)
		test_num += 1

run_tests(2)