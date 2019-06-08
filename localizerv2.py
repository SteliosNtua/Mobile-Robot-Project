#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Float64
import numpy as np

class Logger:
    def __init__(self,name_list):
        self.pub = dict()
        for n in name_list:
            self.pub[n] = rospy.Publisher(n, Float64, queue_size=10)

    def publish(self, plot_list):
        for t in plot_list:
            self.pub[t[0]].publish(t[1])

def KalmanUpdate(mu,S,u,z,A,B,C,Q,R):
    mu_ = np.matmul(A , mu) + np.matmul(B, u)
    # print mu_.shape
    mu[2] = mu[2] % 2*np.pi
    S_ = np.matmul(A,np.matmul(S , A.T)) + R
    # print S_.shape
    K = np.matmul(np.matmul(S_ , C.T),np.linalg.inv( np.matmul(np.matmul(C , S_) , C.T) + Q))
    # print K.shape
    mu_new = mu_ + np.matmul(K ,  z - np.matmul(C , mu_))
    # print mu_new.shape
    S_new = np.matmul((np.eye(5) - np.matmul(K , C) ),S_)
    # print S_new.shape
    #print("mu_:\n{}\nS_:\n{}\nK:\n{}\nmu_new:\n{}\nS_new:\n{}".format(mu_,S_, K,mu_new,S_new))
    return mu_new, S_new

# Sonars:
sonarF_val = 0.0
sonarFL_val = 0.0
sonarFR_val = 0.0
sonarL_val = 0.0
sonarR_val = 0.0
# IMU:
imuRoll = 0.0 # orientation
imuPitch = 0.0
imuYaw = 0.0
imuAngVelX = 0.0 # angular velocity
imuAngVelY = 0.0
imuAngVelZ = 0.0
imuLinAccX = 0.0 # linear acceleration
imuLinAccY = 0.0
imuLinAccZ = 0.0
# input
u_lin = 0.0
u_ang = 0.0
# estimated parameters
estimRoll = 0.0 # always zero value in 2D navigation
estimPitch = 0.0 # always zero value in 2D navigation
estimYaw = 0.0
x = 1.0 * np.zeros((5,1))
x[-1] = 2.4336293856408275
S = np.zeros((5,5))
# Sm = Smoother(3)


ekf_estimation_msg = Odometry()
ekf_estimation_msg.header.frame_id = "odom"
ekf_estimation_msg.child_frame_id = "chassis"


def solve_system(px,py,theta):

    # print px.shape, py.shape, theta.shape
    # y-py = tan(theta)(x-px)
    # NOTE: px = px + 0.1 cos(theta)
    if np.abs(np.sin(theta)) <= np.sqrt(2)/2 :
        tan = np.tan(theta)
        y1 = tan*(2-px) + py
        y2 = tan*(-2-px) + py
        x1 = (2-py)/tan + px
        x2 = (-2-py)/tan + px
    else:
        cot = np.tan(np.pi/2 - theta)
        y1 = (2-px)/cot + py
        y2 = (-2-px)/cot + py
        x1 = (2-py)*cot + px
        x2 = (-2-py)*cot + px

    #solutions
    s1 = np.array([2, y1],dtype=np.float64)
    s2 = np.array([-2, y2],dtype=np.float64)
    s3 = np.array([x1, 2],dtype=np.float64)
    s4 = np.array([x2, -2],dtype=np.float64)
    n = np.array([np.cos(theta), np.sin(theta)],dtype=np.float64)

    solutions = [s1, s2, s3, s4]
    # print n.shape
    # print s1.shape
    # print '***'
    print solutions
    solutions = [s for s in solutions if np.matmul(s,n) >= -0.5]
    print solutions
    solutions = [s for s in solutions if np.max(np.abs(s)) <= 3.0]
    print solutions
    sol = solutions[0]
    if np.sqrt((sol[0]-px)**2 + (sol[1]-py)**2) > 2:
        sol = np.array([px,py]) + 2 * np.array([np.cos(theta),np.sin(theta)])
    return sol

    #Sonar_front = root( (tan(theta)**2 + 1) * (x-px)**2 )
    #l_front = (x-px) / np.cos(theta)
    #l_left = (x-px) / np.cos(np.pi/4 - theta)
    #l_right = (x-px) / np.cos(np.pi/4 + theta)


def send_velocity():
    ekf_pub = rospy.Publisher('/ekf_estimation', Odometry, queue_size=1)

    ekf_estimation_msg.header.seq += 1
    t1 = ekf_estimation_msg.header.stamp
    ekf_estimation_msg.header.stamp = rospy.Time.now()

    DT = ekf_estimation_msg.header.stamp - t1
    DT = DT.to_sec()
    """
    PUT YOUR MAIN CODE HERE
    """
    global x
    global S

    r = np.array(  [[0],
                    [0],
                    [-u_lin*np.sin(x[-1])],
                    [u_lin*np.cos(x[-1])],
                    [0]],dtype=np.float64)
    # Ak = np.eye(5) + DT * np.hstack((np.zeros((5,4)), r))

    Ak = np.eye(5)

    Ak[0,0] = 0
    Ak[1,1] = 0

    Bk = np.array([[np.cos(x[-1]),0],
                    [np.sin(x[-1]), 0],
                    [DT * np.cos(x[-1]), 0],
                    [DT * np.sin(x[-1]), 0],
                    [0, DT]],dtype=np.float64)

    # x = np.matmul(Ak,x) + np.matmul(Bk,u)


#UPDATE

    s1 = solve_system(x[2], x[3], x[4]) #front
    s2 = solve_system(x[2], x[3], x[4] - np.pi/4) #front right
    s3 = solve_system(x[2], x[3], x[4] + np.pi/4) #front left
    s4 = solve_system(x[2], x[3], x[4] - np.pi/2) #right
    s5 = solve_system(x[2], x[3], x[4] + np.pi/2) # left



    C = np.array([  [0,0,-(s1[0]-x[2])/np.sqrt((s1[0]-x[2])**2 + (s1[1]-x[3])**2),(s1[1]-x[3])/np.sqrt((s1[0]-x[2])**2 + (s1[1]-x[3])**2),0], #front
                    [0,0,-(s2[0]-x[2])/np.sqrt((s2[0]-x[2])**2 + (s2[1]-x[3])**2),(s2[1]-x[3])/np.sqrt((s2[0]-x[2])**2 + (s2[1]-x[3])**2),0], #front right
                    [0,0,-(s3[0]-x[2])/np.sqrt((s3[0]-x[2])**2 + (s3[1]-x[3])**2),(s3[1]-x[3])/np.sqrt((s3[0]-x[2])**2 + (s3[1]-x[3])**2),0], #front left
                    [0,0,-(s4[0]-x[2])/np.sqrt((s4[0]-x[2])**2 + (s4[1]-x[3])**2),(s4[1]-x[3])/np.sqrt((s4[0]-x[2])**2 + (s4[1]-x[3])**2),0], #right
                    [0,0,-(s5[0]-x[2])/np.sqrt((s5[0]-x[2])**2 + (s5[1]-x[3])**2),(s5[1]-x[3])/np.sqrt((s5[0]-x[2])**2 + (s5[1]-x[3])**2),0], #left
                    [0,0,0,0,1]],dtype=np.float64)

    R = 0.002**2 * np.diag(np.array([np.cos(x[4])**2, np.sin(x[4])**2, DT**2*np.cos(x[4])**2, DT**2*np.sin(x[4])**2, DT**2],dtype=np.float64))

    Q =  np.diag(np.array([0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.002**2],dtype=np.float64))

    z = np.array([[sonarF_val],[sonarFR_val],[sonarFL_val],[sonarR_val],[sonarL_val],[(imuYaw + np.pi + 2.4336293856408275)%(2*np.pi)]],dtype=np.float64)


    u = np.array([[u_lin],[-u_ang]])

    px_prev = x[2]
    py_prev = x[3]

    x,S = KalmanUpdate(x,S,u,z,Ak,Bk,C,Q,R)

    # if(u_lin == 0):
    #     x[2] = px_prev
    #     x[3] = py_prev



    L.publish([('log_imu_yaw',(imuYaw + np.pi + 2.4336293856408275)%(2*np.pi) - np.pi),
        ('log_imu_angvel',imuAngVelZ),
        ('log_uang',u_ang),
        ('log_ulin',u_lin),
        ('log_vx',-x[0]),
        ('log_vy',-x[1]),
        ('log_px',-x[2]),
        ('log_py',-x[3]),
        ('log_theta',x[4] - np.pi),
        ('log_imu_linaccY',imuLinAccY),
        ('log_imu_linaccX',imuLinAccX),
        ('log_odom_vx',odomLinVelX),
        ('log_odom_vy',odomLinVelY),
        ('log_odom_px',odomPosX),
        ('log_odom_py',odomPosY),
        ('log_odom_theta',odomYaw),
        ('log_odom_uang',odomAngVelZ)])

    estimYaw = 0.0 # orientation to be estimated (-pi,pi]
    # position to be estimated
    ekf_estimation_msg.pose.pose.position.x = 0.0
    ekf_estimation_msg.pose.pose.position.y = 0.0
    ekf_estimation_msg.pose.pose.position.z = 0.0
    # RPY to quaternion
    quaternion = tf.transformations.quaternion_from_euler(estimRoll, estimPitch, estimYaw)
    ekf_estimation_msg.pose.pose.orientation.x = quaternion[0]
    ekf_estimation_msg.pose.pose.orientation.y = quaternion[1]
    ekf_estimation_msg.pose.pose.orientation.z = quaternion[2]
    ekf_estimation_msg.pose.pose.orientation.w = quaternion[3]
    # velocities to be estimated
    ekf_estimation_msg.twist.twist.linear.x = 0.0 # x-linear velocity to be estimated
    ekf_estimation_msg.twist.twist.linear.y = 0.0 # y-linear velocity to be estimated
    ekf_estimation_msg.twist.twist.linear.z = 0.0 # always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.x = 0.0 # always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.y = 0.0 # always zero value in 2D navigation
    ekf_estimation_msg.twist.twist.angular.z = 0.0 # angular velocity to be estimated

    """
    OPTIONAL
    in case your extended kalman filter (EKF) is able to estimate covariances,
    fill in the following variables:
    ekf_estimation_msg.pose.covariance Matrix:6x6
    ekf_estimation_msg.twist.covariance Matrix:6x6
    http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
    """

    ekf_pub.publish(ekf_estimation_msg)

def sonarFrontCallback(msg):
    global sonarF_val
    sonarF_val = msg.range;
    #rospy.loginfo("Front Scan %s", sonarF_val)
    send_velocity()

def sonarFrontLeftCallback(msg):
    global sonarFL_val
    sonarFL_val = msg.range

def sonarFrontRightCallback(msg):
    global sonarFR_val
    sonarFR_val = msg.range

def sonarLeftCallback(msg):
    global sonarL_val
    sonarL_val = msg.range

def sonarRightCallback(msg):
    global sonarR_val
    sonarR_val = msg.range

def cmdCallback(msg):
    global u_lin
    global u_ang
    u_lin = msg.linear.x
    u_ang = msg.angular.z

def imuCallback(msg):

    # orientation:: quaternion to RPY (rool, pitch, yaw)
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    global imuRoll
    global imuPitch
    global imuYaw
    (imuRoll, imuPitch, imuYaw) = euler_from_quaternion (orientation_list)

    # angular velocity
    global imuAngVelX
    imuAngVelX = msg.angular_velocity.x
    global imuAngVelY
    imuAngVelY = msg.angular_velocity.y
    global imuAngVelZ
    imuAngVelZ = msg.angular_velocity.z

    # linear acceleration
    global imuLinAccX
    imuLinAccX = msg.linear_acceleration.x
    global imuLinAccY
    imuLinAccY = msg.linear_acceleration.y
    global imuLinAccZ
    imuLinAccZ = msg.linear_acceleration.z


def odomCallback(msg):

    # position
    global odomPosX
    odomPosX = msg.pose.pose.position.x
    global odomPosY
    odomPosY = msg.pose.pose.position.y

    # orientation:: quaternion to RPY (rool, pitch, yaw)
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    global odomRoll
    global odomPitch
    global odomYaw
    (odomRoll, odomPitch, odomYaw) = euler_from_quaternion (orientation_list)

    # angular velocity
    global odomAngVelZ
    odomAngVelZ = msg.twist.twist.angular.z

    # linear velocity
    global odomLinVelX
    odomLinVelX = msg.twist.twist.linear.x
    global odomLinVelY
    odomLinVelY = msg.twist.twist.linear.y

def follower_py():
    # Starts a new node
    rospy.init_node('localizer_node', anonymous=True)
    rospy.Subscriber("sonarFront_scan", Range, sonarFrontCallback)
    rospy.Subscriber("sonarFrontLeft_scan", Range, sonarFrontLeftCallback)
    rospy.Subscriber("sonarFrontRight_scan", Range, sonarFrontRightCallback)
    rospy.Subscriber("sonarLeft_scan", Range, sonarLeftCallback)
    rospy.Subscriber("sonarRight_scan", Range, sonarRightCallback)
    rospy.Subscriber("imu_data", Imu, imuCallback)
    rospy.Subscriber("cmd_vel", Twist, cmdCallback)
    rospy.Subscriber("odom", Odometry, odomCallback)

    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        L = Logger(['log_imu_yaw',
            'log_imu_angvel',
            'log_ulin',
            'log_uang',
            'log_imu_linaccX',
            'log_vx',
            'log_vy',
            'log_px',
            'log_py',
            'log_theta',
            'log_imu_linaccY',
            'log_odom_vx',
            'log_odom_vy',
            'log_odom_px',
            'log_odom_py',
            'log_odom_theta',
            'log_odom_uang'])
        follower_py()
    except rospy.ROSInterruptException: pass
