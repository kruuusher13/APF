#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np


class SonarPotentialField:

    def __init__(self, sonar_topic = '/depth_data'):

        self.sum_potentials_x = 0
        self.sum_potentials_y = 0
        self.Kp = .3#.9#0.05
        self.Kphi = 10#0.35
        self.sonar_sub_name = sonar_topic
        self.velocity_command_topic_name = '/rexrov2/cmd_vel'


        # Sonar Subscriber
        self.sonar_sub = rospy.Subscriber(self.sonar_sub_name, LaserScan, self.callback_sonar)
        # Velocity publisher
        self.steering_signal = rospy.Publisher(self.velocity_command_topic_name, Twist, queue_size=10)

    # Callback for the sonar topic
    def callback_sonar(self, sonar_data):
        rospy.logdebug_once("Scan starts in angle {:.2f} , and ends in angle {:.2f}"
               .format(sonar_data.angle_min*(180/np.pi), sonar_data.angle_max*(180/np.pi)))

        angles = np.linspace(sonar_data.angle_min, sonar_data.angle_max, num=len(sonar_data.ranges))
        potentials_x = np.multiply(1/np.asarray(sonar_data.ranges)**2, np.cos(angles))
        self.sum_potentials_x = np.sum(potentials_x)
        potentials_y = np.multiply(1/np.asarray(sonar_data.ranges)**2, np.sin(angles))
        self.sum_potentials_y = np.sum(potentials_y)
        

    def there_is_obstacle(self, range, min_dist_to_obstacle):        
        if range <= min_dist_to_obstacle:
            obstacle = True
        else:
            obstacle = False
        return obstacle

    def set_velocity_values(self, linear_x_vel):
    
        linear_x = linear_x_vel - self.Kp * self.sum_potentials_x
        # compute the angle between linear velocity and vy generated by potential obstacles
        angle = np.arctan(float(- self.Kp * self.sum_potentials_y/linear_x))
        # Turn
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = self.Kphi*angle

        self.steering_signal.publish(msg)
       



def potential_field():
    rospy.init_node("potential_field_obstacle_avoidance", anonymous=False, log_level=rospy.DEBUG)
    avoid_boi = SonarPotentialField(sonar_topic = '/rexrov2/sss_sonar')

    rate = rospy.Rate(8) # 8hz
    while not rospy.is_shutdown():
        avoid_boi.set_velocity_values(0.9)
        rate.sleep()

if __name__ == '__main__':
    potential_field()