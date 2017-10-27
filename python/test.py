import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Button

freqs = np.arange(2, 20, 3)

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
t = np.arange(0.0, 1.0, 0.001)
s = np.sin(2*np.pi*freqs[0]*t)
l, = plt.plot(t, s, lw=2)

path_points = [];
path_x = [];
path_y = [];

class Index(object):
    ind = 0

    def next(self, event):
        self.ind += 1
        i = self.ind % len(freqs)
        ydata = np.sin(2*np.pi*freqs[i]*t)
        l.set_ydata(ydata)
        plt.draw()

    def prev(self, event):
        self.ind -= 1
        i = self.ind % len(freqs)
        ydata = np.sin(2*np.pi*freqs[i]*t)
        l.set_ydata(ydata)
        plt.draw()

    def onclick(self, event):
        if event.button and event.inaxes and event.y > 0.04:
            print event.xdata, event.ydata
            # dx, dy = event.inaxes.transData.inverted().transform((event.x, event.y))
            # x = int(dx)
            # y = int(dy)

            path_points.append((event.xdata,event.ydata))
            path_x.append(event.xdata)
            path_y.append(event.ydata)
            plt.plot(path_x, path_y, "-", linewidth = 3, color="red")
            #point = plt.Circle((event.xdata, event.ydata), 2, color="red")
            # fig = plt.gcf()
            # fig.gca().add_artist(point)
            fig.canvas.draw()
            fig.canvas.flush_events()

callback = Index()
axprev = plt.axes([0.7, 0.05, 0.1, 0.075])
axnext = plt.axes([0.81, 0.05, 0.1, 0.075])
bnext = Button(axnext, 'Next')
bnext.on_clicked(callback.next)
bprev = Button(axprev, 'Previous')
bprev.on_clicked(callback.prev)

fig.canvas.mpl_connect('motion_notify_event', callback.onclick)

plt.show()