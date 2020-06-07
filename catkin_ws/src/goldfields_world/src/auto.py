#! /usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
from math import atan
import time

class Move_to_Point(object):
    def __init__(self):
        self.x=0.0
        self.y=0.0
        self.theta=0.0
        rospy.init_node('Control_coordinates')
        self.sub=rospy.Subscriber("/gazebo/model_states",ModelStates,self.NewOdom)
        self.pub=rospy.Publisher('/robot_mov/diff_drive_controller/cmd_vel',Twist,queue_size=1)
        self.PALA_pub=rospy.Publisher('/robot_mov/pala_position_controller/command',Float64,queue_size=1)
        self.Supp_pub=rospy.Publisher('/robot_mov/arm_position_controlller/command',Float64,queue_size=1)
        self.arm=Float64()
        self.pala=Float64()
        self.speed=Twist()
        self.goal= Point()
        self.goal.x=0
        self.goal.y=0
        self.dec=False



    def NewOdom(self,msg): #Recibir quarnetions de gazebo para leerlos como euler
        self.x=msg.pose[1].position.x
        self.y=msg.pose[1].position.y
        rot_q= msg.pose[1].orientation
        (roll, pitch, self.theta)= euler_from_quaternion([rot_q.x, rot_q.y , rot_q.z , rot_q.w])
        
        #Self.theta = Angulo de rotacion en Z

    def Load(self):        #Funcion de cargar con la pala
        self.arm.data=0
        self.pala.data=-1.25
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        time.sleep(1)
        self.arm.data=0.4
        self.pala.data=-0.75
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        time.sleep(1)
        self.pala.data=-0.5
        self.arm.data=0.75
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        time.sleep(1)
        self.pala.data=-0.25
        self.arm.data=1.5
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        for i in range(1,15):
            self.speed.linear.x=-1.5
            self.speed.angular.z=0.0
            self.pub.publish(self.speed)
            time.sleep(0.25)

    def Download(self):   #Funcion para
        self.arm.data=1.5
        self.pala.data=-2.25
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        time.sleep(3)
        self.arm.data=1.0
        self.pala.data=0.0
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        time.sleep(1)
        self.pala.data=0.0
        self.arm.data=0.5
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        time.sleep(1)
        self.pala.data=0.0
        self.arm.data=0.0
        self.PALA_pub.publish(self.pala)
        self.Supp_pub.publish(self.arm)
        for i in range(1,15):
            self.speed.linear.x=-2.0
            self.speed.angular.z=0.0
            self.pub.publish(self.speed)
            time.sleep(0.25)

    def Turn_back(self):
        for i in range(1,45):
            self.speed.linear.x=0.0
            self.speed.angular.z=0.5
            self.pub.publish(self.speed)
            time.sleep(0.5)

    def Turn(self):
        for i in range(1,45):
            self.speed.linear.x=0.0
            self.speed.angular.z=0.5
            self.pub.publish(self.speed)
            time.sleep(0.5)



    def Process_move(self,goal_x,goal_y):
        status=True
        i=0
        while status:
            if i>=5:
                self.goal.x=goal_x
                self.goal.y=goal_y            

                inc_x =  self.goal.x - self.x
                inc_y =  self.goal.y - self.y
                angle_to_goal =-atan(inc_x /inc_y)

                print angle_to_goal
                print self.theta

                if  angle_to_goal < 0 and angle_to_goal -self.theta < 0.001 and self.dec==False:
                    self.speed.linear.x=0.0
                    self.speed.angular.z=-0.5

                elif  angle_to_goal > 0 and angle_to_goal -self.theta > 0.001 and self.dec==False:
                        self.speed.linear.x=0.0
                        self.speed.angular.z=0.5
                    
                else:
                    self.speed.linear.x=3.0
                    self.speed.angular.z=0.0
                    self.dec=True

                if self.y - self.goal.y < 0.001 and self.dec==True:
                    self.dec=False
                    status=False
                    self.speed.linear.x=0.0

                self.pub.publish(self.speed)
            i=i+1

    def Process_move_back(self,goal_x,goal_y):
        status=True
        i=0
        while status:
            if i>=5:
                self.goal.x=goal_x
                self.goal.y=goal_y

                inc_x =  self.goal.x - self.x
                inc_y =  self.goal.y - self.y
                angle_to_goal =atan(inc_x /inc_y)

                if self.theta<0:
                    data=-self.theta-3.1415922654

                if self.theta>0:
                    data=-self.theta+3.1415922654

                print angle_to_goal
                print data


                if  angle_to_goal < 0 and  angle_to_goal+data > angle_to_goal*1.99 and self.dec==False:
                    self.speed.linear.x=0.0
                    self.speed.angular.z=0.5

                elif  angle_to_goal > 0 and angle_to_goal+data < angle_to_goal*1.99 and self.dec==False:
                    self.speed.linear.x=0.0
                    self.speed.angular.z=-0.5

                else:
                    self.speed.linear.x=3.0
                    self.speed.angular.z=0.0
                    self.dec=True

                if self.y - self.goal.y > 0.001 and self.dec==True:
                    self.dec=False
                    status=False
                    self.speed.linear.x=0.0


                self.pub.publish(self.speed)
            i=i+1        


        
    def counter(self):
        return self.count


if __name__=="__main__":
    time.sleep(5)

    mov=Move_to_Point()

    while True:
        mov.Process_move(8.5,5)
        mov.Load()
        mov.Process_move(5.5,-4)
        mov.Process_move(6,-16)
        mov.Process_move(17,-44)
        mov.Process_move(18.25,-54)
        mov.Process_move(18,-61)
        mov.Process_move(18.25,-95)
        mov.Download()
        mov.Turn()
        mov.Process_move_back(17.75,-65)
        mov.Process_move_back(21,-56)
        mov.Process_move_back(18,-34)
        mov.Process_move_back(5,-18)
        mov.Process_move_back(4.5,12)
        mov.Turn()


















        