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

        self.order = ["Attrium",
        "Attrium Door 1",
        "CS Office Door",
        "CS Office",
        "CS Office Door",
        "East Room Door 1",
        "East Room",
        "East Room Door 2",
        "West Room Door 1",
        "West Room",
        "West Room Door 2",
        "Electrical Lab Door 1",
        "Electrical Lab",
        "Electrical Lab Door 2",
        "CS Department Door",
        "CS Department",
        "CS Department Door",
        "Attrium Door 2"]

        rospy.init_node('commentary', anonymous=True)
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
        for i in self.list.curselection():
            self.points.append(coordinates[self.order[i]])
        print(self.points)
        self.giveNext(None)
    
    def giveNext(self, msg):
        if len(self.points) > 0:
            toPublish = Point(0, 0, 0)
            toPublish.x = self.points[0][0]
            toPublish.y = self.points[0][1]
            self.out.publish(toPublish)
            del self.points[0]
        

if __name__ == "__main__":
    Commentary()