import rospy
import json
from std_msgs.msg import String
 
def callback(data):
    color_data = json.loads(data.data)
    print(color_data[0]['color'])

rospy.init_node('color_subscriber')
rospy.Subscriber("color_location", String, callback)
print ("Waiting for messages ..")

try:
    rospy.spin()
except KeyboardInterrupt:
    sys.exit(0)
