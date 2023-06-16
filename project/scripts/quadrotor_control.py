#!/usr/bin/env python3
from math import pi, sqrt, atan2, cos, sin
from turtle import position
import numpy as np
from numpy import NaN
import rospy
import tf
from std_msgs.msg import Empty, Float32
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import Twist, Pose2D
import pickle
import os
class Quadrotor():    
    def __init__(self):
        # publisher for rotor speeds
        self.motor_speed_pub = rospy.Publisher("/crazyflie2/command/motor_speed", Actuators, queue_size=10)
        # subscribe to Odometry topic
        self.odom_sub = rospy.Subscriber("/crazyflie2/ground_truth/odometry",Odometry, self.odom_callback)
        self.t0 = None
        self.t = None
        self.t_series = []
        self.x_series = []
        self.y_series = []
        self.z_series = []

        # System parameters
        self.m = 27e-3
        self.l = 46e-3
        self.Ix = 16.57171e-6
        self.Iy = 16.57171e-6
        self.Iz = 29.261652e-6
        self.Ip = 12.65625e-8
        self.kF = 1.28192e-8
        self.kM = 5.964552e-3
        self.wMax = 2618
        self.wMin = 0
        self.g = 9.81
        #Trajectory Variables Initialisation 
        self.x_des = 0
        self.xdot_des = 0
        self.xddot_des = 0
        self.y_des = 0
        self.ydot_des = 0
        self.yddot_des = 0
        self.z_des = 0
        self.zdot_des = 0
        self.zddot_des = 0
        # Propellor angular velocity
        self.w1 = 0
        self.w2 = 0
        self.w3 = 0
        self.w4 = 0

        

        self.mutex_lock_on = False
        rospy.on_shutdown(self.save_data)
        # TODO: include initialization codes if needed
        
    def traj_evaluate(self):
        # TODO: evaluating the corresponding trajectories designed in Part 1 to return the desired positions, velocities and accelerations

        if self.t<=5:
            ti = 0
            tf = 5
            T = np.array([[1, ti, ti**2, ti**3, ti**4, ti**5],
                [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
                [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
            A = np.dot(np.linalg.inv(T),np.array([[0,0,0],[0,0,0],[0,0,0],[0,0,1],[0,0,0],[0,0,0]]))
            time_mat = np.array([1, self.t, self.t**2, self.t**3, self.t**4, self.t**5])
            time_matdot = np.array([0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4])
            time_matdotdot = np.array([0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3])
            # Position x,y and z 
            self.x_des = np.dot(A[:,0], time_mat)
            self.y_des = np.dot(A[:,1], time_mat)
            self.z_des = np.dot(A[:,2], time_mat)
            # Velocity x,y and z 
            self.xdot_des = np.dot(A[:,0], time_matdot)
            self.ydot_des = np.dot(A[:,1], time_matdot)
            self.zdot_des = np.dot(A[:,2], time_matdot)
            # Acceleration x,y and z 
            self.xddot_des = np.dot(A[:,0], time_matdotdot)          
            self.yddot_des = np.dot(A[:,1], time_matdotdot)
            self.zddot_des = np.dot(A[:,2], time_matdotdot)

        elif self.t<=20:
            ti = 5
            tf = 20
            T = np.array([[1, ti, ti**2, ti**3, ti**4, ti**5],
                [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
                [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
            A = np.dot(np.linalg.inv(T),np.array([[0,0,1],[0,0,0],[0,0,0],[1,0,1],[0,0,0],[0,0,0]]))
            time_mat = np.array([1, self.t, self.t**2, self.t**3, self.t**4, self.t**5])
            time_matdot = np.array([0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4])
            time_matdotdot = np.array([0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3])
            # Position x,y and z 
            self.x_des = np.dot(A[:,0], time_mat)
            self.y_des = np.dot(A[:,1], time_mat)
            self.z_des = np.dot(A[:,2], time_mat)
            # Velocity x,y and z 
            self.xdot_des = np.dot(A[:,0], time_matdot)
            self.ydot_des = np.dot(A[:,1], time_matdot)
            self.zdot_des = np.dot(A[:,2], time_matdot)
            # Acceleration x,y and z 
            self.xddot_des = np.dot(A[:,0], time_matdotdot)          
            self.yddot_des = np.dot(A[:,1], time_matdotdot)
            self.zddot_des = np.dot(A[:,2], time_matdotdot)
            
        elif self.t<=35:
            ti = 20
            tf = 35
            T = np.array([[1, ti, ti**2, ti**3, ti**4, ti**5],
                [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
                [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
            A = np.dot(np.linalg.inv(T),np.array([[1,0,1],[0,0,0],[0,0,0],[1,1,1],[0,0,0],[0,0,0]]))
            time_mat = np.array([1, self.t, self.t**2, self.t**3, self.t**4, self.t**5])
            time_matdot = np.array([0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4])
            time_matdotdot = np.array([0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3])
            # Position x,y and z 
            self.x_des = np.dot(A[:,0], time_mat)
            self.y_des = np.dot(A[:,1], time_mat)
            self.z_des = np.dot(A[:,2], time_mat)
            # Velocity x,y and z 
            self.xdot_des = np.dot(A[:,0], time_matdot)
            self.ydot_des = np.dot(A[:,1], time_matdot)
            self.zdot_des = np.dot(A[:,2], time_matdot)
            # Acceleration x,y and z 
            self.xddot_des = np.dot(A[:,0], time_matdotdot)          
            self.yddot_des = np.dot(A[:,1], time_matdotdot)
            self.zddot_des = np.dot(A[:,2], time_matdotdot)
            
        elif self.t<=50:
            ti = 35
            tf = 50
            T = np.array([[1, ti, ti**2, ti**3, ti**4, ti**5],
                [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
                [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
            A = np.dot(np.linalg.inv(T),np.array([[1,1,1],[0,0,0],[0,0,0],[0,1,1],[0,0,0],[0,0,0]]))
            time_mat = np.array([1, self.t, self.t**2, self.t**3, self.t**4, self.t**5])
            time_matdot = np.array([0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4])
            time_matdotdot = np.array([0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3])
            # Position x,y and z 
            self.x_des = np.dot(A[:,0], time_mat)
            self.y_des = np.dot(A[:,1], time_mat)
            self.z_des = np.dot(A[:,2], time_mat)
            # Velocity x,y and z 
            self.xdot_des = np.dot(A[:,0], time_matdot)
            self.ydot_des = np.dot(A[:,1], time_matdot)
            self.zdot_des = np.dot(A[:,2], time_matdot)
            # Acceleration x,y and z 
            self.xddot_des = np.dot(A[:,0], time_matdotdot)          
            self.yddot_des = np.dot(A[:,1], time_matdotdot)
            self.zddot_des = np.dot(A[:,2], time_matdotdot)
            
        elif self.t<=65:
            ti = 50
            tf = 65
            T = np.array([[1, ti, ti**2, ti**3, ti**4, ti**5],
                [0, 1, 2*ti, 3*ti**2, 4*ti**3, 5*ti**4],
                [0, 0, 2, 6*ti, 12*ti**2, 20*ti**3],
                [1, tf, tf**2, tf**3, tf**4, tf**5],
                [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4],
                [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3]])
            A = np.dot(np.linalg.inv(T),np.array([[0,1,1],[0,0,0],[0,0,0],[0,0,1],[0,0,0],[0,0,0]]))
            time_mat = np.array([1, self.t, self.t**2, self.t**3, self.t**4, self.t**5])
            time_matdot = np.array([0, 1, 2*self.t, 3*self.t**2, 4*self.t**3, 5*self.t**4])
            time_matdotdot = np.array([0, 0, 2, 6*self.t, 12*self.t**2, 20*self.t**3])
            # Position x,y and z 
            self.x_des = np.dot(A[:,0], time_mat)
            self.y_des = np.dot(A[:,1], time_mat)
            self.z_des = np.dot(A[:,2], time_mat)
            # Velocity x,y and z 
            self.xdot_des = np.dot(A[:,0], time_matdot)
            self.ydot_des = np.dot(A[:,1], time_matdot)
            self.zdot_des = np.dot(A[:,2], time_matdot)
            # Acceleration x,y and z 
            self.xddot_des = np.dot(A[:,0], time_matdotdot)          
            self.yddot_des = np.dot(A[:,1], time_matdotdot)
            self.zddot_des = np.dot(A[:,2], time_matdotdot)

    def smc_control(self, xyz, xyz_dot, rpy, rpy_dot):
        # obtain the desired values by evaluating the corresponding
        # trajectories  
        self.traj_evaluate()
        # Tuning parameters
        kp = 95#105
        kd = 6
        lmbda1 = 8 
        lmbda2 = 15
        lmbda3 = 17
        lmbda4 = 6
        phi_b1 = 0.9
        phi_b2 = 0.9
        phi_b3 = 0.9
        phi_b4 = 0.9
        k1 = 12
        k2 = 155
        k3 = 115
        k4 = 23

        Sigma = self.w1 - self.w2 + self.w3 - self.w4
              

        Fx = self.m*(-kp*(xyz[0,0]-self.x_des)-kd*(xyz_dot[0,0]-self.xdot_des)+self.xddot_des)
        Fy = self.m*(-kp*(xyz[1,0]-self.y_des)-kd*(xyz_dot[1,0]-self.ydot_des)+self.yddot_des)

        # Required params
        zd = self.z_des
        dotzd = self.zdot_des
        dotdotzd = self.zddot_des
        # TODO: implement the Sliding Mode Control laws designed in Part 2 to
        # calculate the control inputs "u"
        e1 = xyz[2,0] - zd
        e1_dot = xyz_dot[2,0] - dotzd
        s1 = e1_dot + lmbda1*e1
        sat1 = np.minimum(np.maximum(s1/phi_b1,-1),1)
        u1 = (self.m/(cos(rpy[0,0])*cos(rpy[1,0])))*(self.g + dotdotzd - lmbda1*e1_dot - k1*sat1)
        #----------------------------------------------------------------
        # Required params
        thetad = np.arcsin(Fx/u1)
        phid = np.arcsin(-Fy/u1)
        #----------------------------------------------------------------
        e2 = np.arctan2(np.sin(rpy[0,0]-phid),np.cos(rpy[0,0]-phid))
        e2_dot = rpy_dot[0,0]
        s2 = e2_dot + lmbda2*e2
        sat2 = np.minimum(np.maximum(s2/phi_b2,-1),1)
        u2 = self.Ix*((-rpy_dot[1,0]*rpy_dot[2,0]*(self.Iy - self.Iz)/self.Ix)+(self.Ip*Sigma*rpy_dot[1,0]/self.Ix)-lmbda2*e2_dot-k2*sat2) 
        #----------------------------------------------------------------------------
        e3 = np.arctan2(np.sin(rpy[1,0]-thetad),np.cos(rpy[1,0]-thetad))
        e3_dot = rpy_dot[1,0]
        s3 = e3_dot + lmbda3*e3
        sat3 = np.minimum(np.maximum(s3/phi_b3,-1),1)
        u3 = self.Iy*((-rpy_dot[0,0]*rpy_dot[2,0]*(self.Iz - self.Ix)/self.Iy)-(self.Ip*Sigma*rpy_dot[0,0]/self.Iy)-lmbda3*e3_dot-k3*sat3)
        #------------------------------------------------------------------------------------
        e4 = np.arctan2(np.sin(rpy[2,0]),np.cos(rpy[2,0]))
        e4_dot = rpy_dot[2,0]
        s4 = e4_dot + lmbda4*e4
        sat4 = np.minimum(np.maximum(s4/phi_b4,-1),1)
        u4 = self.Iz*((-rpy_dot[0,0]*rpy_dot[1,0]*(self.Ix - self.Iy)/self.Iz)-lmbda4*e4_dot-k4*sat4)
        #--------------------------------------------------------------------------------------------------
        u = np.array([u1,u2,u3,u4])
        # print("control input",u)
        # REMARK: wrap the roll-pitch-yaw angle errors to [-pi to pi]

        # TODO: convert the desired control inputs "u" to desired rotor
        # velocities "motor_vel" by using the "allocation matrix"
        allocation_matrix = np.array([[(1/(4*self.kF)), (-sqrt(2)/(4*self.kF*self.l)), (-sqrt(2)/(4*self.kF*self.l)), (-1/(4*self.kM*self.kF))],
                                      [(1/(4*self.kF)), (-sqrt(2)/(4*self.kF*self.l)), (sqrt(2)/(4*self.kF*self.l)), (1/(4*self.kM*self.kF))],
                                      [(1/(4*self.kF)), (sqrt(2)/(4*self.kF*self.l)), (sqrt(2)/(4*self.kF*self.l)), (-1/(4*self.kM*self.kF))],
                                      [(1/(4*self.kF)), (sqrt(2)/(4*self.kF*self.l)), (-sqrt(2)/(4*self.kF*self.l)), (1/(4*self.kM*self.kF))]])
        motor_vel = np.sqrt(np.dot(allocation_matrix,np.transpose(u)))
        # TODO: maintain the rotor velocities within the valid range of [0 to
        # 2618]
        for i in range(len(motor_vel)):
            if motor_vel[i] > self.wMax:
                motor_vel[i] = self.wMax
            elif motor_vel[i] < self.wMin:
                motor_vel[i] = self.wMin

        self.w1 = motor_vel[0]
        self.w2 = motor_vel[1]
        self.w3 = motor_vel[2]
        self.w4 = motor_vel[3]
        # print("prop speeds",motor_vel)
        # publish the motor velocities to the associated ROS topic
        motor_speed = Actuators()
        motor_speed.angular_velocities = [motor_vel[0], motor_vel[1],motor_vel[2], motor_vel[3]]
        self.motor_speed_pub.publish(motor_speed)
    # odometry callback function (DO NOT MODIFY)
    def odom_callback(self, msg):
        if self.t0 == None:
            self.t0 = msg.header.stamp.to_sec()
        self.t = msg.header.stamp.to_sec() - self.t0
        # convert odometry data to xyz, xyz_dot, rpy, and rpy_dot
        w_b = np.asarray([[msg.twist.twist.angular.x], [msg.twist.twist.
                                                        angular.y], [msg.twist.twist.angular.z]])
        v_b = np.asarray([[msg.twist.twist.linear.x], [msg.twist.twist.linear.
                                                    y], [msg.twist.twist.linear.z]])
        xyz = np.asarray([[msg.pose.pose.position.x], [msg.pose.pose.position.
                                                    y], [msg.pose.pose.position.z]])
        q = msg.pose.pose.orientation
        T = tf.transformations.quaternion_matrix([q.x, q.y, q.z, q.w])
        T[0:3, 3] = xyz[0:3, 0]
        R = T[0:3, 0:3]
        xyz_dot = np.dot(R, v_b)
        rpy = tf.transformations.euler_from_matrix(R, 'sxyz')
        rpy_dot = np.dot(np.asarray([
            [1, np.sin(rpy[0])*np.tan(rpy[1]), np.cos(rpy[0])*np.tan(rpy[1])],
            [0, np.cos(rpy[0]), -np.sin(rpy[0])],
            [0, np.sin(rpy[0])/np.cos(rpy[1]), np.cos(rpy[0])/np.cos(rpy[1])]
            ]), w_b)
        rpy = np.expand_dims(rpy, axis=1)
        # store the actual trajectory to be visualized later
        if (self.mutex_lock_on is not True):
            self.t_series.append(self.t)
            self.x_series.append(xyz[0, 0])
            self.y_series.append(xyz[1, 0])
            self.z_series.append(xyz[2, 0])
        # call the controller with the current states
        self.smc_control(xyz, xyz_dot, rpy, rpy_dot)
    # save the actual trajectory data
    def save_data(self):
        # TODO: update the path below with the correct path
        with open("/home/chaitanya/rbe502_project/src/project/scripts/log.pkl",
                "wb") as fp:
            self.mutex_lock_on = True
            pickle.dump([self.t_series,self.x_series,self.y_series,self.
                        z_series], fp)
if __name__ == '__main__':
    rospy.init_node("quadrotor_control")
    rospy.loginfo("Press Ctrl + C to terminate")
    whatever = Quadrotor()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")