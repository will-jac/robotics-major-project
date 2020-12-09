import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from locations import *

import read_initial_position


class TaskPlanner():
    def __init__(self):
        rospy.init_node('task_planner', anonymous=False)

        args = rospy.myargv()
        x, y, z = read_initial_position.get_initial_pose(args)
        self.init = Point()
        self.init.x = x
        self.init.y = y
        self.init.z = z
        rospy.loginfo('Please enter which rooms you would like to visit')
        rospy.loginfo('Accepted commands are all, atrium, office, east, west, electrical, and department')
        rospy.loginfo('Enter commands once per line')
        rospy.loginfo('Enter blank line when you are done selecting')

        rospy.on_shutdown(self.shutdown)

        self.publisher = rospy.Publisher('project/task_planner', Point, queue_size=1)

        self.reset()

        rospy.spin()

    def read_input(self, prompt):
        x = input(prompt)
        while x:
            yield x
            x = input(prompt)

    def reset(self):
        self.load_tasks()
        self.next_loc = 0
        rospy.loginfo('sending loc:' + str(self.points[self.next_loc].x) + str(self.points[self.next_loc].y))
        self.publisher.publish(self.points[self.next_loc])
        rospy.loginfo('going to next loc')

    def monitor_update(self, msg):
        if msg.data:
            self.next_loc += 1
            if self.next_loc >= len(self.points):
                rospy.loginfo('End of tour! Please start a new tour')
                self.reset()
            else:
                rospy.loginfo('Failed to reach destination. Please start a new tour')
                self.reset()

    def load_tasks(self):
        self.tasks = []
        input_text = list(self.read_input("->"))
        if 'all' in input_text:
            dest = Point()
            dest.x, dest.y = coordinates["Attrium"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["Attrium Door 1"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["CS Office Door"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["CS Office"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["CS Office Door"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["East Room Door 1"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["East Room"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["East Room Door 2"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["West Room Door 1"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["West Room"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["West Room Door 2"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["Electrical Lab Door 1"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["Electrical Lab"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["Electrical Lab Door 2"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["CS Department Door"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["CS Department"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["CS Department Door"]
            self.tasks.append(dest)
            dest.x, dest.y = coordinates["Attrium Door 2"]
            self.tasks.append(dest)
        else:
            if 'atrium' in input_text:
                dest = Point()
                dest.x, dest.y = coordinates["Attrium"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["Attrium Door 1"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["Attrium Door 2"]
                self.tasks.append(dest)
            if 'office' in input_text:
                dest = Point()
                dest.x, dest.y = coordinates["CS Office Door"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["CS Office"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["CS Office Door"]
                self.tasks.append(dest)
            if 'east' in input_text:
                dest = Point()
                dest.x, dest.y = coordinates["East Room Door 1"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["East Room"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["East Room Door 2"]
                self.tasks.append(dest)
            if 'west' in input_text:
                dest = Point()
                dest.x, dest.y = coordinates["West Room Door 1"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["West Room"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["West Room Door 2"]
                self.tasks.append(dest)
            if 'electrical' in input_text:
                dest = Point()
                dest.x, dest.y = coordinates["Electrical Lab Door 1"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["Electrical Lab"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["Electrical Lab Door 2"]
                self.tasks.append(dest)
            if 'department' in input_text:
                dest = Point()
                dest.x, dest.y = coordinates["CS Department Door"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["CS Department"]
                self.tasks.append(dest)
                dest.x, dest.y = coordinates["CS Department Door"]
                self.tasks.append(dest)

    def shutdown(self):
        rospy.loginfo('stopping task planner')




