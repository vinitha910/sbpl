import yaml
import sys
import csv 

hbsp_stream = open("/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility/scenarios/house/hbsp_stats.yaml", "r")
hbsp_data = yaml.load(hbsp_stream)

dijkstra_stream = open("/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility/scenarios/house/dijkstra_stats.yaml", "r")
dijkstra_data = yaml.load(dijkstra_stream)

hd_stream = open("/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility/scenarios/house/hbsp_stats_3.yaml", "r")
hd_data = yaml.load(hd_stream)

hbsp_hard_stream = open("/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility/scenarios/house/hbsp_stats_hard.yaml", "r")
hbsp_hard_data = yaml.load(hbsp_hard_stream)

dijkstra_hard_stream = open("/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility/scenarios/house/dijkstra_stats_hard.yaml", "r")
dijkstra_hard_data = yaml.load(dijkstra_hard_stream)

hd_hard_stream = open("/home/vinitha910/workspaces/mhi_ws/src/sbpl_mobility/scenarios/house/hbsp_stats_3_hard.yaml", "r")
hd_hard_data = yaml.load(hd_hard_stream)

test_num = 0
with open("stats.csv", 'w') as f:
	writer = csv.writer(f, lineterminator='\n')
	writer.writerow(["type", "test", "total_planning_time", "time_creating_heuristics"])
	while test_num < 23:
		test = "test_" + str(test_num)
		if test in hbsp_data:
			hbsp_pt = hbsp_data[test]['total_planning_time']
			hbsp_h_time = hbsp_data[test]['time_creating_heuristics']
			writer.writerow(["hbsp_1", test_num, hbsp_pt, hbsp_h_time])
		test_num += 1

	test_num = 0
	while test_num < 23:
		test = "test_" + str(test_num)
		if test in hd_data:	
			hbsp_3_pt = hd_data[test]['total_planning_time']
			hbsp_3_h_time = hd_data[test]['time_creating_heuristics']
			writer.writerow(["hbsp_3", test_num, hbsp_3_pt, hbsp_3_h_time])
		test_num += 1

	test_num = 0
	while test_num < 23:
		test = "test_" + str(test_num)
		if test in dijkstra_data:
			d_pt = dijkstra_data[test]['total_planning_time']
			d_h_time = dijkstra_data[test]['time_creating_heuristics']
			writer.writerow(["dijkstra", test_num, d_pt, d_h_time])
		test_num += 1

	test_num = 0
	while test_num < 21:
		test = "test_" + str(test_num)
		if test in hbsp_hard_data:
			hbsp_pt = hbsp_hard_data[test]['total_planning_time']
			hbsp_h_time = hbsp_hard_data[test]['time_creating_heuristics']
			writer.writerow(["hbsp_1_hard", test_num, hbsp_pt, hbsp_h_time])
		test_num += 1

	test_num = 0
	while test_num < 21:
		test = "test_" + str(test_num)
		if test in hd_hard_data:	
			hbsp_3_pt = hd_hard_data[test]['total_planning_time']
			hbsp_3_h_time = hd_hard_data[test]['time_creating_heuristics']
			writer.writerow(["hbsp_3_hard", test_num, hbsp_3_pt, hbsp_3_h_time])
		test_num += 1

	test_num = 0
	while test_num < 21:
		test = "test_" + str(test_num)
		if test in dijkstra_hard_data:
			d_pt = dijkstra_hard_data[test]['total_planning_time']
			d_h_time = dijkstra_hard_data[test]['time_creating_heuristics']
			writer.writerow(["dijkstra_hard", test_num, d_pt, d_h_time])
		test_num += 1