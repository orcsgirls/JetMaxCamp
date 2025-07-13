import rospy
import json
from std_msgs.msg import String
 
def callback(data):
    tag_data = json.loads(data.data)
    if len(tag_data)>0:
        print(tag_data[0]['id'])
    else:
        print('No data')

rospy.init_node('apriltag_subscriber')
rospy.Subscriber("apriltag_location", String, callback)
print ("Waiting for messages ..")

try:
    rospy.spin()
except KeyboardInterrupt:
    sys.exit(0)
