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
        self.arm=Float64()
        self.pala=Float64()
        self.goal= Point()
        self.speed=Twist()
        self.goal.x=0
        self.goal.y=0
        self.dec=False
        self.count=0
        self.once=True
        self.angle_med=0

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
        self.count+=1

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
        self.count+=1

    def Get_pose_in_gazebo(self): #Funcion para llamar al service para obtener posicion y angulo
        model_coordinates=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        object_coordinates= model_coordinates("robot_mov","")
        x_position= object_coordinates.pose.position.x
        y_position= object_coordinates.pose.position.y

        rot_q= object_coordinates.pose.orientation
        (roll, pitch, theta)= euler_from_quaternion([rot_q.x, rot_q.y , rot_q.z , rot_q.w])

        Pose=[x_position,y_position,theta]
        
        return Pose

    def Move_Front(self,goal_x,goal_y):
        self.goal.x=goal_x
        self.goal.y=goal_y

        Pose=self.Get_pose_in_gazebo()
        x_var= Pose[0] - self.goal.x
        y_var= Pose[1] - self.goal.y
        angle_to_goal=-math.atan2(x_var,y_var)

        if self.once:
            self.angle_med=angle_to_goal
            self.once=False
        
        if angle_to_goal < Pose[2] and self.dec==False:  #Girar Derecha
            self.speed.linear.x=0.0
            #Velocidades dependiendo del angulo
            if Pose[2] - angle_to_goal < 0.19365:
                self.speed.angular.z=-0.25
            elif 0.19365 < Pose[2] - angle_to_goal < 0.7854:
                self.speed.angular.z=-0.5
            elif 0.7854 < Pose[2] - angle_to_goal < 1.5708:
                self.speed.angular.z=-0.75
            elif Pose[2] - angle_to_goal > 1.5708:
                self.speed.angular.z=1.0

        elif angle_to_goal > Pose[2]+0.001 and self.dec==False:    #Girar izquierda #Se agrega el 0.001 para que haya un invervalo entre Derecha y Izquierda
            self.speed.linear.x=0.0
            #Velocidades dependiendo del angulo
            if angle_to_goal - Pose[2] < 0.19635:
                self.speed.angular.z=0.25
            elif 0.19365 < angle_to_goal - Pose[2] < 0.7854:
                self.speed.angular.z=0.5
            elif 0.7854 < angle_to_goal - Pose[2] < 1.5708:
                self.speed.angular.z=0.75
            elif angle_to_goal - Pose[2] > 1.5708:
                self.speed.angular.z=1.0

        else:
            self.speed.angular.z=0.0
            if abs(y_var)<10:
                self.speed.linear.x=1.0
            elif 5<abs(y_var)<10:
                self.speed.linear.x=2.0
            elif abs(y_var)>10:
                self.speed.linear.x=3.0
            self.dec=True

        if self.dec==True:
            if Pose[1] <= self.goal.y and -1.5708<self.angle_med<1.5708:
                self.dec=False
                self.once=True
                self.speed.linear.x=0.0
                self.count+=1

            elif self.angle_med<-1.5708 or self.angle_med>1.5708:
                if Pose[1] >= self.goal.y:
                    self.dec=False
                    self.once=True
                    self.speed.linear.x=0.0
                    self.count+=1


        self.pub.publish(self.speed)

    def Num(self):
        return self.count

    def Reset(self):
        self.count=0

        

    



    
if __name__=="__main__":
    Follower=Follow_Point()
    ctrl_c=False

    def shutdownhook():
        global ctrl_c
        ctrl_c=True
        rospy.loginfo("Cancelled Node")

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        print Follower.Num()
        if Follower.Num() == 0:
            Follower.Move_Front(9,4.5)
        if Follower.Num() == 1:
            Follower.Load()
        if Follower.Num() == 2:
            Follower.Move_Front(5,-4)
        if Follower.Num() == 3:
            Follower.Move_Front(5.25,-17)
        if Follower.Num() == 4:
            Follower.Move_Front(16,-34)
        if Follower.Num() == 5:
            Follower.Move_Front(21.5,-58)
        if Follower.Num() == 6:
            Follower.Move_Front(18,-62)
        if Follower.Num() == 7:
            Follower.Move_Front(18,-100)
        if Follower.Num() == 8:
            Follower.Download()
        if Follower.Num() == 9:
            Follower.Move_Front(18,-62)
        if Follower.Num() == 10:
            Follower.Move_Front(21.5,-58)
        if Follower.Num() == 11:
            Follower.Move_Front(16,-34)
        if Follower.Num() == 12:
            Follower.Move_Front(5.25,-17)
        if Follower.Num() == 13:
            Follower.Move_Front(5,-4)
        if Follower.Num() == 14:
            Follower.Move_Front(6,15)
        if Follower.Num() == 15:
            Follower.Reset()
        

