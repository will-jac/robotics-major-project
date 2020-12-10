import rospy
from geometry_msgs.msg import Point
from locations import *
from playsound import playsound

class Commentary():

    def __init__(self):
        rospy.init_node('commentary', anonymous=True)
        rospy.on_shutdown(self.shutdown)
        rospy.Subscriber('/project/pose', Point, self.update_pos)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def update_pos(self, point):
        x = round(point.x)
        y = round(point.y)
        for key in commentaryMap.keys():
            if round(coordinates[key][0]) == x and round(coordinates[key][1]) == y:
                print("Playing some commentary")
                playsound(commentaryMap[key], True)
                return

    def shutdown(self):
        playsound(None)
        print("Fine, I'll shut up")

if __name__ == "__main__":
    Commentary()