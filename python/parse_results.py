import yaml
import sys

hbsp_stream = open(sys.argv[1], "r")
hbsp_data = yaml.load(hbsp_stream)

dijkstra_stream = open(sys.argv[2], "r")
dijkstra_data = yaml.load(dijkstra_stream)

hd_stream = open(sys.argv[3], "r")
hd_data = yaml.load(hd_stream)

test_num = 1
num_tests = 0
num_hbsp_sol = 0
num_dijkstra_sol = 0
num_hd_sol = 0
hbsp_sum_times = 0
dijkstra_sum_times = 0
hd_sum_times = 0
hbsp_h_time = 0
dijkstra_h_time = 0
hd_h_time = 0

while test_num < 80:
	test = "test_" + str(test_num)
	if test in hbsp_data:
		if (int(hbsp_data[test]['solution_found']) == 1):
			num_hbsp_sol += 1
			hbsp_sum_times += float(hbsp_data[test]['total_planning_time'])
			hbsp_h_time += float(hbsp_data[test]['time_creating_heuristics'])
		if (int(dijkstra_data[test]['solution_found']) == 1):
			num_dijkstra_sol += 1
			dijkstra_sum_times += float(dijkstra_data[test]['total_planning_time'])
			dijkstra_h_time += float(dijkstra_data[test]['time_creating_heuristics'])
		if (int(hd_data[test]['solution_found']) == 1):
			num_hd_sol += 1
			hd_sum_times += float(hd_data[test]['total_planning_time'])
			hd_h_time += float(hd_data[test]['time_creating_heuristics'])
		num_tests += 1

	test_num += 1

hbsp_avg_time = hbsp_sum_times/num_hbsp_sol
dijkstra_avg_time = dijkstra_sum_times/num_dijkstra_sol
hd_avg_time = hd_sum_times/num_hd_sol

hbsp_h_avg = hbsp_h_time/num_hbsp_sol
dijkstra_h_avg = dijkstra_h_time/num_dijkstra_sol
hd_h_avg = hd_h_time/num_hd_sol

print ""
print "MHA* using HBSP Heuristic (1 signature):"
print "    Number of Solutions Found:           " + str(num_hbsp_sol) + "/" + str(num_tests) 
print "    Average Heuristic Construction Time: " + str(hbsp_h_avg) 
print "    Average Planning Time:               " + str(hbsp_avg_time) + "s\n"

print "MHA* using HBSP Heuristic (3 signatures):"
print "    Number of Solutions Found:           " + str(num_hd_sol) + "/" + str(num_tests) 
print "    Average Heuristic Construction Time: " + str(hd_h_avg)
print "    Average Planning Time:               " + str(hd_avg_time) + "s\n"

print "MHA* using only anchor 2D Dijkstra Heuristic:"
print "    Number of Solutions Found:           " + str(num_dijkstra_sol) + "/" + str(num_tests)
print "    Average Heuristic Construction Time: " + str(dijkstra_h_avg) 
print "    Average Planning Time:               " + str(dijkstra_avg_time) + "s\n"

print "Percent Decrease in Planning Time with HBSP Heuristic: " + str((dijkstra_avg_time - hd_avg_time)/dijkstra_avg_time * 100) + "%\n"