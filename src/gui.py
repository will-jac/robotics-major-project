#!/usr/bin/env python
from locations import *

from Tkinter import *
import tkMessageBox
import Tkinter
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Empty

class Commentary():

    def __init__(self):
        self.points = []

        self.order = ["Atrium",
        "CS Office",
        "East Room",
        "West Room",
        "Electrical Lab",
        "CS Department"]

        rospy.init_node('gui', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        top = Tk()
        top.geometry("500x420")
        top.title("Choose your destinations!")

        self.list = Listbox(top, selectmode=MULTIPLE)
        for i in self.order:
            self.list.insert(END, i)
        go = Button(top, text="Go to selected locations", command=self.prepOutput)

        self.list.pack(side=TOP, fill=BOTH, expand=True)
        go.pack(side=BOTTOM, fill=X, expand=False)
        
        rospy.Subscriber('/project/task_request', Empty, self.giveNext)
        self.out = rospy.Publisher('/project/task_planner', Point, queue_size=1)

        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            top.update()

    def shutdown(self):
        print("Closing thing")

    def prepOutput(self):
        self.points = []
        visited_indexes = []
        curr_index = 0
        min_index = -1
        min_dist = -1
        for i in self.list.curselection():
            if len(visited_indexes) == 0:
                self.points.append(coordinates[self.order[i]])
                visited_indexes.append(self.order.index(self.order[i]))
            else:
                dist_array = adjacency[self.order[i]]
                curr_index = index[self.order[i]]
                for j in range(6):
                    if j not in visited_indexes and j != curr_index and j in self.list.curselection():
                        if min_dist == -1:
                            min_dist = dist_array[j]
                            min_index = j
                        else:
                            if dist_array[j] < min_dist:
                                min_dist = dist_array[j]
                                min_index = j
                if min_index != -1:
                    self.points.append(coordinates[self.order[min_index]])
                    visited_indexes.append(min_index)
                    min_index = -1
                    min_dist = -1
        self.points.append((0, 0))
        print(self.points)
        self.giveNext(None)
    
    def giveNext(self, msg):
        if len(self.points) > 0:
            toPublish = Point(0, 0, 0)
            toPublish.x = self.points[0][0]
            toPublish.y = self.points[0][1]
            self.out.publish(toPublish)
            del self.points[0]
            print(self.points)
        

if __name__ == "__main__":
    Commentary()
