
from gpiozero import DistanceSensor
import rospy
from std_msgs.msg import Bool
from sensor_msgs.msg import Range

class SonarDevice():

    def __init__(self, name, trigger, echo, range_min, threshold, range_max, angle_min_deg, angle_max_deg):
        self.name = name
        self.range_min = range_min
        self.range_max = range_max
        self.threshold = threshold
        self.fov = math.radians(angle_max_deg-angle_min_deg)
        self.sensor = DistanceSensor(echo=echo, trigger=trigger, max_distance=range_max)

        topic_base = "/jayrover/sonar/{}"

        rate_topic_name = topic_base.format(name)
        self.rate_pub = rospy.Publisher(rate_topic_name, Range, queue_size=1)

        threshold_topic_name = topic_base.format(name + "_threshold")
        self.threshold_pub = rospy.Publisher(threshold_topic_name, Bool, queue_size=10)
        self.threshold_state = False

        rospy.loginfo("Publisher set with topic {}".format(topic_name))

    def scan(self):

        range_val = self.sensor.distance
        
        message = Range()
        message.radiation_type = 0
        message.field_of_view = self.fov
        message.min_range = self.range_min
        message.max_range = self.range_max
        message.range = range_val

        self.rate_pub.publish(message)
        if((not self.threshold_state) and range_val < self.threshold):
            self.threshold_pub.publish(True)
            self.threshold_state = True
        elif(self.threshold_state and range_val >= self.threshold):
            self.threshold_pub.publish(False)
            self.threshold_state = False

        rospy.loginfo("Range {} [m]: {} (in threshold: {})".format(self.name, range_val, self.threshold_state)
