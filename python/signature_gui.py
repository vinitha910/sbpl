import Tkinter as tk
from Tkinter import *
import sys
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('TkAgg')
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from PIL import ImageTk

class ExampleApp(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        self.previous_x = self.previous_y = 0
        self.x = self.y = 0
        self.points_recorded = []
        self.path = []
        self.canvas = tk.Canvas(self, width=500, height=500, bg = "white", cursor="cross")
        self.canvas.pack(side="top", fill="both", expand=True)
        self.image = ImageTk.PhotoImage(file = "/home/vinitha910/Pictures/projection.png")
        self.canvas.create_image(0, 0, image = self.image, anchor = NW)
        self.button_print = tk.Button(self, text = "Display Path Points", command = self.print_points)
        self.button_print.pack(side="top", fill="both", expand=True)
        self.button_clear = tk.Button(self, text = "Delete Path", command = self.clear_all)
        self.button_clear.pack(side="top", fill="both", expand=True)
        self.button_sig = tk.Button(self, text = "Display Signature", command = self.signature)
        self.button_sig.pack(side="top", fill="both", expand=True)
        self.canvas.bind("<Motion>", self.tell_me_where_you_are)
        self.canvas.bind("<B1-Motion>", self.draw_from_where_you_are)

    def clear_all(self):
        for l in self.path:
            self.canvas.delete(l)
        self.points_recorded = []

    def print_points(self):
        if self.points_recorded:
            for p in self.points_recorded[1:]:
                print p

    def tell_me_where_you_are(self, event):
        self.previous_x = event.x
        self.previous_y = event.y

    def draw_from_where_you_are(self, event):
        if self.points_recorded:
            self.points_recorded.pop()

        self.x = event.x
        self.y = event.y
        l = self.canvas.create_line(self.previous_x, self.previous_y, 
                                self.x, self.y,fill="yellow")
        self.path.append(l)
        self.points_recorded.append((self.previous_x, 500 - self.previous_y))
        self.points_recorded.append((self.x, 500 - self.y))       
        self.previous_x = self.x
        self.previous_y = self.y

    def signature(self):
        centroids = [(183, 150), (40, 115), (114, 262), (168, 208), (143, 152),
                     (132, 254), (218, 73), (269, 215), (330, 65)]
        prev = self.points_recorded[0]
        sig = []
        for p in self.points_recorded[1:]:
            curr = p
            for i in range(0,len(centroids)):
                if curr[0] > centroids[i][0] and curr[1] > centroids[i][1]:
                    if prev[0] <= centroids[i][0]: 
                        sig.append(i+1)
                elif prev[0] > centroids[i][0] and prev[1] > centroids[i][1]:
                    if curr[0] <= centroids[i][0]:  
                        sig.append(-i-1)

                # for i in range(0,len(sig)-2):
                #     if sig[i] == -1*sig[i+1]:
                #         del sig[i]
                #         del sig[i+1]
            prev = curr
        print sig

if __name__ == "__main__":
    app = ExampleApp()
    app.mainloop()