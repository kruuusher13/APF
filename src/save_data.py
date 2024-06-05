import rospy
import csv

from dsor_msgs.msg import Measurement

# Callback function to handle the received message
def callback(msg):
    # Extract the timestamp and values
    timestamp = msg.header.stamp.secs
    frame_id = msg.header.frame_id
    value = msg.value
    noise = msg.noise

    # Save the data to CSV file
    # with open('output.csv', 'a') as file:
    #     writer = csv.writer(file)
    #     writer.writerow([timestamp, frame_id] + value + noise)
 
    # print("Data saved to CSV file.")
    # print(f"Timestamp: {timestamp}")
    # print(f"Frame ID: {frame_id}")
    # print(f"Value: {value}")
    # print(f"Noise: {noise}")

    if (frame_id=="bluerov_heavy0_altimeter"):
         print(f"Value: {value}")

# Initialize the ROS node
rospy.init_node('listener', anonymous=True)

# Subscribe to the topic
rospy.Subscriber('/bluerov_heavy0/measurement/position', Measurement, callback)

# Spin ROS
rospy.spin()
