import sys
import re
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import yaml
from random import randint

sys.setrecursionlimit(10000)

class DrawPath():

    def __init__(self, map, start_x, start_y, end_x, end_y, centroids):
        self.map = map
        self.map_values = self.get_map(self.map)
        self.fig, self.ax = plt.subplots(figsize=(10,10))
        self.ax.set_title('Draw a path from the blue circle to the green circle')
        self.ax.imshow(self.map_values, vmin=0, vmax=1, cmap='Greys', extent=None, aspect='equal')

        plt.ylim([0, self.map_values.shape[1]])
        plt.xlim([0, self.map_values.shape[0]])

        self.start_x = start_x
        self.start_y = start_y
        self.end_x = end_x
        self.end_y = end_y
        self.centroids = centroids

        self.start_circle = plt.Circle((end_x,end_y), 5, color='blue')
        self.goal_circle = plt.Circle((start_x,start_y), 5, color='green')
        self.ax.add_artist(self.start_circle)
        self.ax.add_artist(self.goal_circle)

        self.axb1 = plt.axes([0.21, 0.03, 0.2, 0.05])
        self.sigb1 = Button(self.axb1, 'Display Signature')
        self.sigb1.on_clicked(self.display_signature)

        self.axb2 = plt.axes([0.442, 0.03, 0.14, 0.05])
        self.sigb2 = Button(self.axb2, 'Delete Path')
        self.sigb2.on_clicked(self.delete_path)

        self.axb3 = plt.axes([0.615, 0.03, 0.2, 0.05])
        self.sigb3 = Button(self.axb3, 'Print Path Points')
        self.sigb3.on_clicked(self.print_path_points)

        self.fig.canvas.mpl_connect('motion_notify_event', self.onclick)

        self.path_points = []
        self.path_x = []
        self.path_y = []

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
        return map_values
                
    def display_signature(self, event):
        prev = self.path_points[0]
        sig = []
        for p in self.path_points[1:]:
            curr = p
            for i in range(0,len(self.centroids)):
                if curr[0] > self.centroids[i][0] and curr[1] > self.centroids[i][1]:
                    if prev[0] <= self.centroids[i][0]: 
                        sig.append(i+1)
                elif prev[0] > self.centroids[i][0] and prev[1] > self.centroids[i][1]:
                    if curr[0] <= self.centroids[i][0]:  
                        sig.append(-i-1)
            prev = curr
        print sig

    def delete_path(self, event):
        self.path_x = []
        self.path_y = []
        self.path_points = []

        self.fig_plot.set_xdata(self.path_x)
        self.fig_plot.set_ydata(self.path_y)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def print_path_points(self, event):
        print self.path_points

    def onclick(self, event):
        if event.button and event.inaxes:
            self.path_points.append((int(event.xdata), int(event.ydata)))
            self.path_x.append(event.xdata)
            self.path_y.append(event.ydata)

            self.fig_plot.set_xdata(self.path_x)
            self.fig_plot.set_ydata(self.path_y)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

if __name__ == '__main__':
    stream = open(sys.argv[1], "r")
    params = yaml.load(stream)
    end_x = params['goal']['GoalX'] / 0.05
    end_y = params['goal']['GoalY'] / 0.05
    start_x = params['StartX'] / 0.05
    start_y = params['StartY'] / 0.05

    centroids = [(202,168), (56, 142), (48,112), (136, 90), (195, 209), (166, 265),
                 (153, 164), (232, 273), (277, 79), (330, 281), (363, 191), (384, 245), (403, 144)]
    #centroids = [(17, 9), (5,142), (74,0), (117,70), (90,110), (161,12)]

    draw_path = DrawPath(sys.argv[2], start_x, start_y, end_x, end_y, centroids)

      

        