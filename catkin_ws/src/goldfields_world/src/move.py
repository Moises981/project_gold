#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist,Point
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64
import math
import time



class Follow_Point(object):
    def __init__(self):
        rospy.init_node("Pose")
        self.pub=rospy.Publisher('/robot_mov/diff_drive_controller/cmd_vel',Twist,queue_size=1)
        self.PALA_pub=rospy.Publisher('/robot_mov/pala_position_controller/command',Float64,queue_size=1)
        self.Supp_pub=rospy.Publisher('/robot_mov/arm_position_controlller/command',Float64,queue_size=1)
        self.dec=False
        self.goal= Point()
        self.speed=Twist()
        self.goal.x=8
        self.goal.y=5.5
        self.dec=False
        self.count=0

    def Get_pose_in_gazebo(self): #Funcion para llamar al service para obtener posicion y angulo
        model_coordinates=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        object_coordinates= model_coordinates("robot_mov","")
        x_position= object_coordinates.pose.position.x
        y_position= object_coordinates.pose.position.y

        rot_q= object_coordinates.pose.orientation
        (roll, pitch, theta)= euler_from_quaternion([rot_q.x, rot_q.y , rot_q.z , rot_q.w])

        Pose=[x_position,y_position,theta]
        
        return Pose

    def Move_Front(self):
        Pose=self.Get_pose_in_gazebo()
        x_var= Pose[0] - self.goal.x
        y_var= Pose[1] - self.goal.y
        angle_to_goal=-math.atan2(x_var,y_var)

        if angle_to_goal < 0 and angle_to_goal <= Pose[2] and self.dec==False:

            self.speed.linear.x=0.0

            if angle_to_goal <=-1.5708:  #Giro rapido
                self.speed.angular.z=-0.75

            if -0.78 >= angle_to_goal >=-1.57: #Giro medio
                self.speed.angular.z=-0.5
            
            if angle_to_goal>=-1.57: #Giro lento
                self.speed.angular.z=-0.25

        elif angle_to_goal > 0 and angle_to_goal >= Pose[2] and self.dec==False:
            self.speed.linear.x=0.0
            self.speed.angular.z=0.25

        else:
            self.speed.linear.x=3.0
            self.speed.angular.z=0.0
            self.dec=True

        if self.dec==True:
            if Pose[1] >= self.goal.y and self.goal.y > 0:    
                self.dec=False
                self.speed.linear.x=0.0
                self.count+=1
                print "D"

            elif Pose[1] <= self.goal.y and self.goal.y < 0:    
                self.dec=False
                self.speed.linear.x=0.0
                self.count+=1
                print "N"


        self.pub.publish(self.speed)

    def Num(self):
        return self.count
        

    



    
if __name__=="__main__":
    Follower=Follow_Point()
    ctrl_c=False

    def shutdownhook():
        global ctrl_c
        ctrl_c=True
        rospy.loginfo("Cancelled Node")

    while not ctrl_c:
        if Follower.Num() == 0:
            Follower.Move_Front()

