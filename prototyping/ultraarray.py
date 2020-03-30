#!/usr/bin/python

from ultrareading import Sonar
import math
import rospy
from sensor_msgs.msg import Range


#------- PARAMETERS
NUM_SONAR                   = 2

#-- RIGHT
SONAR_RIGHT_GPIO_TRIGGER     = 27
SONAR_RIGHT_GPIO_ECHO        = 22

#-- LEFT
SONAR_LEFT_GPIO_TRIGGER      = 23
SONAR_LEFT_GPIO_ECHO         = 24

class SonarArray():
    def __init__(self,
            num_sonar,
            gpio_trigger_list,   #-- list of all the trigger pins, starting from left
            gpio_echo_list,      #-- list of all the echo pins, starting from left
            range_min,          #- [m]
            range_max,          #- [m]
            angle_min_deg,      #- [deg]
            angle_max_deg
            ):

        self.sonar_array    = []
        self.pub_array      = []
        self.num_sonar      = num_sonar

        delta_angle_deg = (angle_max_deg-angle_min_deg)/float(num_sonar-1)

        rospy.loginfo("Initializing the arrays")
        #--- Create an array and expand the object with its angle
        for i in range(num_sonar):
            sonar       = Sonar(gpio_trigger_list[i], gpio_echo_list[i], range_min=range_min*100, range_max=range_max*100)
            angle_deg   = angle_min_deg + delta_angle_deg*i
            sonar.angle = math.radians(angle_deg)
            self.sonar_array.append(sonar)
            rospy.loginfo("Sonar %d set"%i)

            #--- Publishers
            topic_name  = "/jayrover/sonar/%d"%i
            pub = rospy.Publisher(topic_name, Range, queue_size=5)
            self.pub_array.append(pub)
            rospy.loginfo("Publisher %d set with topic %s"%(i, topic_name))

        #--- Default message
        message = Range()
        message.radiation_type  = 0
        message.min_range       = range_min
        message.max_range       = range_max
        self._message           = message


    def scan(self):

        range_array = []
        for i in range(self.num_sonar):
            range_cm = self.sonar_array[i].get_range()
            range_array.append(range_cm*0.01)
            self._message.range         = range_cm*0.01
            self._message.field_of_view = self.sonar_array[i].angle     #-- put the angle in field of view
            self.pub_array[i].publish(self._message)

        rospy.loginfo("Range [m]: left = %4.2f  center = %4.2f right = %4.2f"%(range_array[0],
            range_array[1], range_array[2]))

    def run(self):
        #--- Set the control rate
        rate = rospy.Rate(10)

        rospy.loginfo("Running...")
        while not rospy.is_shutdown():
            self.scan()
            rate.sleep()

        rospy.loginfo("Stopped")

if __name__ == "__main__":

    rospy.loginfo("Setting Up the Sonar Node...")

    rospy.init_node('sonar_array')

    sonar_array     = SonarArray(num_sonar          = NUM_SONAR,
                                 gpio_trigger_list  = [SONAR_LEFT_GPIO_TRIGGER, SONAR_RIGHT_GPIO_TRIGGER],
                                 gpio_echo_list     = [SONAR_LEFT_GPIO_ECHO, SONAR_RIGHT_GPIO_ECHO],
                                 range_min          = 0.2,
                                 range_max          = 2,
                                 angle_min_deg      = -15,
                                 angle_max_deg      = 15
                                    )
    sonar_array.run()
