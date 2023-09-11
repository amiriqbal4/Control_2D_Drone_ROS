#!/usr/bin/env python

import rospy
import math
import matplotlib.pyplot as plt
from geometry_msgs.msg import Pose2D, Twist
import numpy as np

class Drone2D:
    def __init__(self, x=0.0, y=0.0, theta=0.0,vx=0.0,vy=0.0,w=0.0, xd=1.0, yd=0.0, theta_d=math.pi/2, vd=1.0, Rd=1.0):
        ### Initial state ###
        self.x = x
        self.y = y
        self.theta = theta

        ### Initial velocity ###
        self.vx = vx
        self.vy = vy
        self.w = w

        ### Initial desired state ###
        self.xd = xd
        self.yd = yd
        self.theta_d = theta_d   

        ### Desired heading speed and circle radius ###
        self.vd = vd
        self.Rd=Rd   

        ### For integral control law
        self.integral_error_x=0.0
        self.integral_error_y=0.0
        self.integral_error_theta=0.0

        ### Initialize ROS node, publisher, and subscriber ###
        rospy.init_node('drone_control_2D', anonymous=True)

        ### 1. Publisher: publishes desired pose on topic /desired_pose
        self.pose_publisher = rospy.Publisher('/desired_pose', Pose2D, queue_size=20) 

        ### 2. Subscriber: subscribes to /desired_pose trajectory andcomputes control in control_callback
        rospy.Subscriber('/desired_pose', Pose2D, self.control_callback) 

        ### Optional publisher: Publishes the control command on topic '/closed_loop_vel' that can be mapped to an actual hardware
        self.control_publisher = rospy.Publisher('/closed_loop_vel', Twist, queue_size=20) 
    
        self.rate = rospy.Rate(10)  # 10 Hz
        self.dt =1/50
        self.sim_time =0.0
        print(self.dt)

    ### A method to numerically update current state based on commanded vx, vy, vz, and step time dt ###
    ### This input comes from the callback function (feedback control loop) ###
    def update_current_state(self,dt):
        self.x += self.vx * dt
        self.y += self.vy * dt
        self.theta += self.w * dt # yaw rate W = v/R

        ### Numerically update desired trajectory based on desired speed, Radius, and the steptime dt ####
    def update_desired_state(self, dt):
        self.theta_d += (self.vd /self.Rd) * dt # yaw rate W = v/R   
        self.xd += self.vd  * math.cos(self.theta_d) * dt
        self.yd += self.vd  * math.sin(self.theta_d) * dt
 
        ### Returns current sate of the drone at the current simulation instant ###
    def get_current_state(self):
        return self.x, self.y, self.theta
    
        ### Returns desiried sate of the drone  at the current simulation instant ###
    def get_desired_state(self):
        return self.xd, self.yd, self.theta_d   

    def control_callback(self,control_msg):
        # Proportional and derivative gains for feedback control
        kp_x = 2.0  #  gain for x position error
        kp_y = 2.0  #  gain for y position error
        kp_theta = 1.0  # gain for theta (yaw) error

        kd_x = 0.50  #  gain for vx error
        kd_y = 0.50  #  gain for vy error
        kd_theta = 0.1  # gain w error    

        ki_x = 0.5  #  gain for cumulative vx error
        ki_y = 0.5  #  gain for cumulative vy error
        ki_theta = 0.1  # gain  cumulative w error   
        
        # Get current and desired states
        x_curr, y_curr, th_curr = self.get_current_state()
        x_des, y_des, th_des = self.get_desired_state()
        
        # Calculate position and yaw errors
        x_error = x_des - x_curr
        y_error = y_des - y_curr
        theta_error = th_des - th_curr

        self.integral_error_x+=x_error
        self.integral_error_y+=y_error
        self.integral_error_theta+=theta_error

        # Calculate velocity  errors
        vx_error = self.vd * math.cos(self.theta_d) - self.vx
        vy_error = self.vd * math.sin(self.theta_d) - self.vy
        w_error = self.vd/self.Rd - self.w
        
        # Calculate control inputs using feedback gains and errors
        u_x = kp_x * x_error + kd_x*vx_error +ki_x*self.integral_error_x
        u_y = kp_y * y_error + kd_y*vy_error + ki_y*self.integral_error_y
        u_w = kp_theta * theta_error + kd_theta*w_error + ki_theta*self.integral_error_theta
        
        # Update desired closed-loop velocity commands
        self.vx += u_x
        self.vy += u_y
        self.w += u_w
        
        # Update control message with desired velocity commands
        control_msg =Twist()
        control_msg.linear.x = self.vx
        control_msg.linear.y = self.vy
        control_msg.angular.z = self.w
        
        # # Publish the updated control message
        self.control_publisher.publish(control_msg)


    def start_drone_simulation(self):
        # Initialize arrays to store actual and desired trajectory points
        time = []
        actual_trajectory = [] #  x, y, th
        desired_trajectory = [] # x_d, y_d, th_d
        while not rospy.is_shutdown() and self.sim_time<10: # simulation stops if t>10s
            # Get desired state
            x_des, y_des, th_des = self.get_desired_state()

            # Get actual state
            x_actual, y_actual, th_actual = self.get_current_state()

            # Create Pose2D message for desired pose publishing
            pose_msg = Pose2D()
            pose_msg.x = x_des
            pose_msg.y = y_des
            pose_msg.theta = th_des

            # Publish the desired pose
            self.pose_publisher.publish(pose_msg)

            time.append(self.sim_time)
            actual_trajectory.append([x_actual, y_actual, th_actual])
            desired_trajectory.append([x_des, y_des, th_des])
            print([self.sim_time, x_actual, y_actual, th_actual])

            self.update_desired_state(self.dt)  # Update the desired state trajectory
            self.update_current_state(self.dt)
            self.sim_time+=self.dt
            # self.rate.sleep()
            rospy.sleep(self.dt)
        # Plot the actual and desired trajectories
        actual_trajectory =np.array(actual_trajectory)
        desired_trajectory =np.array(desired_trajectory)
        figure1 = plt.figure(figsize=(6, 6))
        plt.plot(actual_trajectory[:,0], actual_trajectory[:,1], label="Actual")
        plt.plot(desired_trajectory[:,0], desired_trajectory[:,1], label="Desired")
        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")
        plt.title("Actual vs Desired Trajectory")
        plt.legend()
        plt.grid()
        plt.show()
        figure2 = plt.figure(figsize=(8, 4))
        plt.plot(time, actual_trajectory[:,0], label="Actual")
        plt.plot(time, desired_trajectory[:,0], label="Desired")
        plt.xlabel("Time (s)")
        plt.ylabel("X (m)")
        plt.title("Actual vs Desired Trajectory")
        plt.legend()
        plt.grid()
        plt.show()

        figure3 = plt.figure(figsize=(8, 4))
        plt.plot(time, actual_trajectory[:,1], label="Actual")
        plt.plot(time, desired_trajectory[:,1], label="Desired")
        plt.xlabel("Time (s)")
        plt.ylabel("Y (m)")
        plt.title("Actual vs Desired Trajectory")
        plt.legend()
        plt.grid()
        plt.show()                   


if __name__ == '__main__':
    drone = Drone2D()
    
    # Initialize a list to store actual and desired trajectory points
    # actual_trajectory = []
    # desired_trajectory = []
    drone.start_drone_simulation()
