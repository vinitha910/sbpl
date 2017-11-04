import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import yaml
from random import randint, uniform
import math
import glob
import os

sys.setrecursionlimit(10000)

class DrawPath():

    def __init__(self, map):
        self.map = map
        self.map_values, self.size_x, self.size_y  = self.get_map(self.map)
        self.fig, self.ax = plt.subplots(figsize=(10,10))
        test_num = len(glob.glob("../../sbpl_mobility/scenarios/house/*")) - 1
        self.ax.set_title("Draw a path from the green circle to the red circle \n Test " + str(test_num), fontsize=18)
        self.ax.imshow(self.map_values, vmin=0, vmax=1, cmap='Greys')
        self.sx, self.sy, self.gx, self.gy = self.get_random_start_goal_pairs()
        self.theta = 0

        plt.ylim([0, self.map_values.shape[1]])
        plt.xlim([0, self.map_values.shape[0]])

        # self.start_x = start_x
        # self.start_y = start_y
        # self.end_x = end_x
        # self.end_y = end_y
        self.centroids = centroids

        self.start_circle = plt.Circle((self.gx,self.gy), 5, color='green')
        self.goal_circle = plt.Circle((self.sx,self.sy), 5, color='red')
        self.ax.add_artist(self.start_circle)
        self.ax.add_artist(self.goal_circle)

        self.axb1 = plt.axes([0.123, 0.03, 0.15, 0.05])
        self.sigb1 = Button(self.axb1, 'Add Signature')
        self.sigb1.on_clicked(self.add_signature)

        self.axb2 = plt.axes([0.3, 0.03, 0.13, 0.05])
        self.sigb2 = Button(self.axb2, 'Delete Path')
        self.sigb2.on_clicked(self.delete_path)

        self.axb3 = plt.axes([0.45, 0.03, 0.265, 0.05])
        self.sigb3 = Button(self.axb3, 'Regenerate Start/Goal Pairs')
        self.sigb3.on_clicked(self.regen_random_start_goal_pairs)

        self.axb4 = plt.axes([0.74, 0.03, 0.158, 0.05])
        self.sigb4 = Button(self.axb4, 'Save Test File')
        self.sigb4.on_clicked(self.write_test_file)

        self.fig.canvas.mpl_connect('motion_notify_event', self.onclick)

        self.path_points = []
        self.path_x = []
        self.path_y = []

        self.sig = []

        self.fig_plot, = self.ax.plot(self.path_x, self.path_y, "-", linewidth = 3, color="red")

        self.ax.axis('off')
        plt.show()

    def get_map(self, filename):
        f = open(filename, 'r')
        lines = f.readlines()
        try:
            map_size_str = [p for p in lines if "discretization" in p][0]
        except IndexError:
            raise Exception("Couldn't find discretization parameter in cfg file.")
     
        try:
            cell_size_str = [p for p in lines if "cellsize" in p][0]
        except IndexError:
            raise Exception("Couldn't find cell size in cfg file")
        
        try:
            size_x, size_y = re.compile('([0-9]+) ([0-9]+)').findall(map_size_str)[0]
            size_x = int(size_x)
            size_y = int(size_y)
        except IndexError:
            raise Exception("Couldn't parse distretization param in cfg file.")
     
        try:
            search = re.compile('([0-9]*\.[0-9]*)')
            cell_size = float(search.findall(cell_size_str)[0])
        except IndexError:
            raise Exception("Couldn't parse cell size")
     
        map_index = [p[0]+1 for p in enumerate(lines) if "environment" in p[1]]
        map_lines = lines[map_index[0]:]
        map_lines = [p.strip().split() for p in map_lines]
        flattened_values = map(int, [item for sublist in map_lines for item in 
                                     sublist])
        map_values = np.array(flattened_values).reshape(size_y, size_x)
        return map_values, size_x, size_y

    def regen_random_start_goal_pairs(self, event):
        if self.start_circle != None:
            self.start_circle.remove()

        if self.goal_circle != None:
            self.goal_circle.remove()

        self.sx, self.sy, self.gx, self.gy = self.get_random_start_goal_pairs()

        self.start_circle = plt.Circle((self.gx,self.gy), 5, color='green')
        self.goal_circle = plt.Circle((self.sx,self.sy), 5, color='red')
        self.ax.add_artist(self.start_circle)
        self.ax.add_artist(self.goal_circle)
        self.delete_path(event)

    def get_random_start_goal_pairs(self):
        max_y = self.size_x - 1
        max_x = 308 # HACK: map can only be square?
        valid_start = False 
        valid_goal = False

        while not valid_start:
            sx = randint(0, max_x)
            sy = randint(0, max_y)
            if self.map_values[sx][sy] == 0:
                valid_start = True

        while not valid_goal:
            gx = randint(0, max_x)
            gy = randint(0, max_y)
            if self.map_values[gx][gy] == 0:
                valid_goal = True

        self.theta = round(uniform(0, math.pi),3)
        return sy, sx, gy, gx

    def add_signature(self, event):
        if self.path_points == []:
            return

        prev = self.path_points[0]
        sig = []
        
        for p in self.path_points[1:]:
            curr = p
            for i in range(0,len(self.centroids)):
                if curr[0] > self.centroids[i][0] and curr[1] > self.centroids[i][1]:
                    if prev[0] <= self.centroids[i][0]: 
                        if len(sig) == 0: 
                            sig.append(i+1)
                        elif sig[len(sig) - 1] == (-i-1):
                            del sig[len(sig) - 1]
                        else:
                            sig.append(i+1)
                elif prev[0] > self.centroids[i][0] and prev[1] > self.centroids[i][1]:
                    if curr[0] <= self.centroids[i][0]:  
                        if len(sig) == 0:
                            sig.append(-i-1)
                        elif sig[len(sig) - 1] == (i+1):
                            del sig[len(sig) - 1]
                        else:    
                            sig.append(-i-1)
            prev = curr

        self.sig.append(sig)
        print self.sig
        self.delete_path(event)

    def delete_path(self, event):
        self.path_x = []
        self.path_y = []
        self.path_points = []

        self.fig_plot.set_xdata(self.path_x)
        self.fig_plot.set_ydata(self.path_y)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def write_test_file(self, event):
        test_num = len(glob.glob("../../sbpl_mobility/scenarios/house/*")) - 1
        
        dir_name_hbsp = "../../sbpl_mobility/scenarios/house/hbsp_test_" + str(test_num)
        if not os.path.exists(dir_name_hbsp):
            os.makedirs(dir_name_hbsp)
        filename_hbsp = dir_name_hbsp + "/planning_params.yaml"
        
        dir_name_dijkstra = "../../sbpl_mobility/scenarios/house/dijkstra_test_" + str(test_num)
        if not os.path.exists(dir_name_dijkstra):
            os.makedirs(dir_name_dijkstra)
        filename_dijkstra = dir_name_dijkstra + "/planning_params.yaml"

        np_sig = np.array(self.sig)
        np_sig_str = np.array2string(np_sig, separator=', ')

        f1 = open(filename_hbsp, "w")
        f1.write("heuristics:\n")
        f1.write("  - type: HomotopicBasedHeuristic\n")
        f1.write("    signatures: " + np_sig_str + "\n")
        f1.write("    visualize: false\n")
        f1.write("\n")
        f1.write("StartX: " + str(float(self.sx)*0.05) + "\n")
        f1.write("StartY: " + str(float(self.sy)*0.05) + "\n")
        f1.write("StartYaw: " + str(float(self.theta)) + "\n")
        f1.write("\n")
        f1.write("goal:\n")
        f1.write("  type: pose\n")
        f1.write("  GoalX: " + str(float(self.gx)*0.05) + "\n")
        f1.write("  GoalY: " + str(float(self.gy)*0.05) + "\n")
        f1.write("  GoalZ: 0.65\n")
        f1.write("  tolerance_xy: 0.3\n")
        f1.write("  tolerance_z: 0.75\n")
        f1.close()

        f2 = open(filename_dijkstra, "w")
        f2.write("heuristics:\n")
        f2.write("  - type: 2DDijkstraHeuristic\n")
        f2.write("    visualize: false\n")
        f2.write("\n")
        f2.write("StartX: " + str(float(self.sx)*0.05) + "\n")
        f2.write("StartY: " + str(float(self.sy)*0.05) + "\n")
        f2.write("StartYaw: " + str(float(self.theta)) + "\n")
        f2.write("\n")
        f2.write("goal:\n")
        f2.write("  type: pose\n")
        f2.write("  GoalX: " + str(float(self.gx)*0.05) + "\n")
        f2.write("  GoalY: " + str(float(self.gy)*0.05) + "\n")
        f2.write("  GoalZ: 0.65\n")
        f2.write("  tolerance_xy: 0.3\n")
        f2.write("  tolerance_z: 0.75\n")
        f2.close()

        self.ax.set_title("Draw a path from the green circle to the red circle \n Test " + str(test_num + 1))
        self.delete_path(event)
        self.regen_random_start_goal_pairs(event)
        self.sig = []

    def print_path_points(self, event):
        print self.path_points

    def onclick(self, event):
        #print self.map_values[int(event.xdata)][int(event.ydata)]
        if event.button and event.inaxes:
            self.path_points.append((int(event.xdata), int(event.ydata)))
            self.path_x.append(event.xdata)
            self.path_y.append(event.ydata)

            self.fig_plot.set_xdata(self.path_x)
            self.fig_plot.set_ydata(self.path_y)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

if __name__ == '__main__':
    centroids = [(202,168), (56, 142), (48,112), (136, 90), (195, 209), (166, 265),
                 (153, 164), (232, 273), (277, 79), (342, 245), (403, 144)]
    #centroids = [(17, 9), (5,142), (74,0), (117,70), (90,110), (161,12)]

    draw_path = DrawPath(sys.argv[1])

      

        