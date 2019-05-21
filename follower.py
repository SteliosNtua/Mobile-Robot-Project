#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Float64
import numpy as np
import sys

pub = rospy.Publisher('test', Float64, queue_size=10)

###### HELPER FUNCTIONS ######

def calc_distance_orientation(x1, x2):
    x3 = np.sqrt( x1**2 + x2**2 -np.sqrt(2)*x1*x2)
    # orient = orientation with respect to the wall
    theta = np.arccos((x1**2 + x3**2 - x2**2)/(2 * x1 * x3))
    orient =  np.pi/2 - theta
    # mu = mid range of triangle formed by the wall the fright and right radar beams
    mu = x1*np.sin(theta)
    return mu, orient

##############################

###### SYSTEM CLASSES ######

class Integrator:
    def __init__(self):
        self.state = 0
        self.time = 0

    def __call__(self, input, new_time):
        self.state = input *(new_time - self.time) + self.state
        self.time = new_time

        return self.state

class Differentiator:
    def __init__(self):
        self.state = 0
        self.time = 0

    def __call__(self, input, new_time):
        ret = (input - self.state) / (new_time - self.time)
        self.time = new_time
        self.state = input
        return ret

class Smoother:
    def __init__(self, order):
        self.pos = 0
        self.order = order
        self.values = [0] * order

    def __call__(self, input):
        self.values[self.pos % self.order] = input
        self.pos += 1
        return np.mean(self.values)

class MinFilter:
    def __init__(self, order):
        self.pos = 0
        self.order = order
        self.values = [0] * order

    def __call__(self, input):
        self.values[self.pos % self.order] = input
        self.pos += 1
        return np.min(self.values)

############################

class Robot:

    def __init__(self, state=0):
        #state 0: find wall
        #state 1: follow wall
        #state 2: turn
        self.state = state
        self.timestamp = 0
        self.updated_sensors = {
                'right': False,
                'fright': False,
                'front': False,
                'fleft': False,
                'left': False
        }
        self.sonar = {
            'right': 0,
            'fright': 0,
            'front': 0,
            'fleft': 0,
            'left': 0
        }

        self.sonar_filter = {
            'right': MinFilter(3),
            'fright': MinFilter(3),
            'front': MinFilter(3),
            'fleft': MinFilter(3),
            'left': MinFilter(3)
        }

        # stage0 system
        self.init_dd = 0.8
        self.K0_lin = -0.3
        self.K0_ang = -0.3
        # stage3 system
        self.int1 = Integrator()
        self.K1 = 5
        # stage2 system
        self.following_linear = 0.3
        self.diff1 = Differentiator()
        self.dd = 0.5
        self.Kp = -1.5
        self.Kd = -0.5
        self.aggresiveness = 1.5
        self.angle_detect_dist = 1.5
        self.angle_detect_thres = np.pi/16
        self.err_smoother = Smoother(5)

    def calculate_action(self):
        msg = Twist()

        # STATE 0
        if self.state == 0: #find wall
            #NOTE: use fleft,fright aswell
            e_lin = self.init_dd - self.sonar['front']
            e_ang = self.sonar['fright'] - self.sonar['fleft']
            #calculate linear velocity
            msg.linear.x = e_lin * self.K0_lin
            #calculate angular velocity
            msg.angular.z = e_ang * self.K0_ang

            #check wall positioning condition
            if(np.abs(e_lin) < 0.1 and np.abs(e_ang) < 0.05):
                msg.linear.x = 0
                msg.angular.z = 0
                rospy.loginfo("Stage0 Completed")
                self.state = 1 # go to turning state
                self.int1.time = self.timestamp

        # STATE 1
        elif self.state == 1: #turn
           msg.linear.x = 0.3
           msg.angular.z = -0.5*(2 - self.sonar['front'])
           if(self.sonar['front'] > 1.80):
               rospy.loginfo("Stage1 Completed")
               self.state = 2

        # STATE 2
        elif self.state == 2: #follow wall
            msg.linear.x = self.following_linear
            mu, theta = calc_distance_orientation(self.sonar['right'], self.sonar['fright'])
            e = theta - self.aggresiveness * (mu - self.dd)*np.pi/4
            e = self.err_smoother(e)
            Pout = self.Kp * e
            Dout = self.Kd * self.diff1(e, self.timestamp)
            out = Pout + Dout
            msg.angular.z = out
            L.publish([('log_error',e), ('log_Pout',Pout), ('log_Dout',Dout), ('log_out', out), ('log_theta', theta), ('log_mu', mu)])
            #if( np.abs(theta) < self.angle_detect_thres and self.sonar['front'] <= self.angle_detect_dist and self.sonar['fleft'] <= self.angle_detect_dist):
            if( np.abs(theta) < self.angle_detect_thres and self.sonar['front'] <= self.angle_detect_dist ):
                rospy.loginfo("Corner Detected, Stage2 Completed")
                rospy.loginfo("theta: %f", theta/np.pi)
                self.state = 1

        # STATE 3
        elif self.state == 3: #parallelize robot with wall
            msg.angular.z = -0.1 * self.K1
            msg.linear.x = 0
            if(self.int1(msg.angular.z, self.timestamp) < -np.pi/2):
                rospy.loginfo("Stage3 Completed")
                msg.angular.z = 0
                self.state = 2
                self.diff1.time = self.timestamp

        else:
            rospy.logerr("Unknown Robot state")

        L.publish([('log_lin_vel',msg.linear.x), ('log_ang_vel',msg.angular.z), ('log_stage', self.state)])
        return msg

    def update_values(self,which,value):
        #rospy.loginfo("Updating %s with: %.2f", which, value)
        self.updated_sensors[which] = True
        self.sonar[which] = value
        if(sum(self.updated_sensors.values()) == 5):
            self.timestamp = rospy.get_time()
            #rospy.loginfo("All sensors read")
            for k in self.updated_sensors.keys():
                self.updated_sensors[k] = False
            msg = self.calculate_action()
            velocity_pub.publish(msg)

class Logger:
    def __init__(self,name_list):
        self.pub = dict()
        for n in name_list:
            self.pub[n] = rospy.Publisher(n, Float64, queue_size=10)

    def publish(self, plot_list):
        for t in plot_list:
            self.pub[t[0]].publish(t[1])

###### CALLBACK FUNCTIONS ###### 

def sonarFrontCallback(msg):
    R.update_values('front', msg.range)

def sonarFrontLeftCallback(msg):
    R.update_values('fleft', msg.range)

def sonarFrontRightCallback(msg):
    R.update_values('fright', msg.range)

def sonarLeftCallback(msg):
    R.update_values('left', msg.range)

def sonarRightCallback(msg):

    R.update_values('right', msg.range)

################################

def follower_py():
    # Starts a new node
    rospy.init_node('follower_node', anonymous=True, log_level=rospy.DEBUG)
    rospy.Subscriber("sonarFront_scan", Range, sonarFrontCallback)
    rospy.Subscriber("sonarFrontLeft_scan", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonarFrontRight_scan", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonarLeft_scan", Range, sonarLeftCallback)
    rospy.Subscriber("sonarRight_scan", Range, sonarRightCallback)
    global velocity_pub
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    if len(sys.argv) == 2:
        R = Robot(int(sys.argv[1]))
    else:
        R = Robot()

    L = Logger(['log_error', 'log_Pout', 'log_Dout', 'log_out', 'log_lin_vel', 'log_ang_vel', 'log_stage', 'log_theta', 'log_mu'])

    try:
        #Testing our function
        follower_py()
    except rospy.ROSInterruptException: pass

