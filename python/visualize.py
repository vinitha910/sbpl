"""
Plots paths outputted from the xytheta example. 
 
Usage: 
    $ python visualize.py [cfg file] [solution file]
 
"""
import sys
import re
import numpy as np
import matplotlib.pyplot as plt
 
def get_map(filename):
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
    map_values = np.fromiter(flattened_values, dtype=np.int).reshape(size_y, size_x)
    return (map_values, cell_size)
 
 
def plot_planned_path(filename, ax, color):
    f = open(filename, 'r')
    lines = f.readlines()
    #print([p.strip().split() for p in lines])
    sol_values = []
    for x in [p.strip().split() for p in lines]:
        sol_values.append(list(map(int, x)))
    sol_values = np.array(sol_values)
    #print(sol_values_lst)
    #sol_values = np.fromiter(sol_values_lst, dtype=np.int)
    print(sol_values)
    ax.plot(sol_values[:,0], sol_values[:,1], color) 
 
def plot_cont_planned_path(filename, cell_size, ax):
    f = open(filename, 'r')
    lines = f.readlines()
    sol_values = np.array([map(float, p.strip().split(',')) for p in lines if
                           len(p.strip().split(',')) == 3])
    ax.plot(sol_values[:,0]/cell_size, sol_values[:,1]/cell_size, 'y-') 
 
if __name__ == '__main__':
    (map_values, cell_size) = get_map(sys.argv[1])
    fig = plt.figure(dpi=300)
    ax = fig.add_subplot(111)
    plt.imshow(map_values, vmin=0, vmax=1, cmap='jet', origin='lower')
    plt.ylim([0, map_values.shape[0]])
    plt.xlim([0, map_values.shape[1]])
 
    plot_planned_path(sys.argv[2], ax, 'y')
    plot_planned_path(sys.argv[3], ax, 'w')
    plot_planned_path(sys.argv[4], ax, 'g')
    #plot_planned_path(sys.argv[5], ax, 'c')
    plt.gca().invert_yaxis()
    plt.show()
