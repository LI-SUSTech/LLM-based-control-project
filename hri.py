#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_kinematics import forward_k, ur_invk

import os
from google.api_core import client_options as client_options_lib


# from google.oauth2 import service_account



# client = palm.Client(credentials=credentials)


import google.generativeai as palm
# from google.api_core import client_options as client_options_lib

PI = 3.1415926535

import json

class SharedControl:
    def __init__(
        self,
        ee_start_xy=(0.3, -0.2),
        ee_goal_xy=(0.3, 0.5),
        ee_z=0.05,
    ):
        # ***********************************************
        # ************ Add whatever you need ************
        # recommended end-effector motion range: x [0.189, 0.304], y [-0.074, 0.296], z [0.05, 0.20]
        self.iter = 0
        api_key = <YOUR KEY HERE>
        palm.configure(
            api_key=api_key,
            transport="rest",
            client_options=client_options_lib.ClientOptions(
                api_endpoint=os.getenv("GOOGLE_API_BASE"),
            )
        )
        
        # print(self.model_bison)
        self.alpha=1 #default is robot policy
        self.path_len = 0
        self.break_flag=0
        self.previous_value=0
        self.scale = 1
        self.thetas = [0,0,0,0,0,0]
        self.ee_next_pos = np.zeros(3)
        self.ee_curr_pos = np.zeros(3)
        self.human_policy = np.zeros(2) # on XY plane
        self.robot_policy = np.zeros(2) # on XY plane
        self.SPIN_RATE = 20
        self.ee_start_pos = np.array([ee_start_xy[0], ee_start_xy[1], ee_z])
        self.ee_goal_pos = np.array([ee_goal_xy[0], ee_goal_xy[1], ee_z])
        self.path = [self.ee_start_pos]
        self.ee_z = ee_z
        self.pub_cmd = rospy.Publisher('ur3/command', command, queue_size=10)
        self.pub_control_clock = rospy.Publisher('control_clock', String, queue_size=10)
        rospy.init_node('shared_control', anonymous=True)
        self.loop_rate = rospy.Rate(self.SPIN_RATE)
        rospy.Subscriber('keyboard_input', String, self.human_policy_callback)
        rospy.Subscriber('control_clock', String, self.shared_control_callback)
        rospy.Subscriber('ur3/position', position, self.position_callback)
        rospy.sleep(1)
        rospy.loginfo("Shared control is initialized.")
        self.go_to_start_pos()
        self.dx = 0
        
        # ***********************************************

    def generate_text( self,prompt):
        models = [m for m in palm.list_models() if 'generateText' in m.supported_generation_methods]
        self.model_bison = models[0]
        # print(self.model_bison)

        
        
        return palm.generate_text(prompt=prompt,
                                 model=self.model_bison,
                                 temperature=0.0)
    
    
    
    def human_policy_callback(self, msg):
        # *****************************************
        # ************ To be done here ************
        x=0
        y=0
        keyboard_input=msg.data
        print('input something you want to control the robot: eg. stop, restart, move x unit away:')
        #these are the code to control the llm prompting
        
        prompt_template = """
            {priming}

            {question}

            Your solution:
            """
        
        # priming_text = "You are a robot assistant who replies list under the following rules \
        #     you do not reply any message just the tuple. your reply is based on the user message. \
        #     If the user relays sentiment to stop the machine you return [0,1], else if the user relays the meassage to start or resume again you ruturn [0,0].\
        #     If the user relays sentiment to move the robot x units close or near, eg. x units close, you return [1,-x]. \
        #     On the contrary, If the user relays sentiment to move the robot x units away, eg. x units away, you return [1,x]. Replace x with the user input number.\
        #     if neither is the case you return [2,]"
            
        priming_text = "You are a robot assistant who replies list under the following rules \
            you do not reply any message just the tuple. your reply is based on the user message. \
            you will be given a sentiment command and the x position of robot, eg.[x]\
            If the user relays sentiment to check the safety, and x is less than 0.28 , you reture [0,2], if x is more than 0.28, you reture [0,3].\
            If the user relays sentiment to stop the machine you return [0,1], else if the user relays the meassage to start or resume again you ruturn [0,0].\
            If the user relays sentiment to move the robot y units close or near, eg. y units close, you return [1,-y]. \
            On the contrary, If the user relays sentiment to move the robot y units away, eg. y units away, you return [1,y]. Replace y with the user input number.\
            if neither is the case you return [2,]\
            If you have nothing to answer, rerutn [3,0]"
            
        print(str(self.ee_curr_pos[0]))
        prompt = prompt_template.format(priming=priming_text,
                                question=keyboard_input +'the x position of robot is'+ str(self.ee_curr_pos[0]) # uisng key board and the x position of end effector as priming
                                )

        # print('test: ',self.generate_text(prompt).result)
        completion_json = json.loads(self.generate_text(prompt).result)
        # print('completion_json: ',completion_json)
        completion = np.array(completion_json)
        
        # print('completion: ', type(completion))
        if completion[1] == 2: # unsafety warning:
            print('-----WARN------')
            print('----UNSAFE------')
        
        if completion[1] == 3: # unsafety warning:
            print('-----WARN------')
            print('----SAFE------')
            
        if completion[0] == 0: #stop the motion
            self.break_flag=completion[1]
            # print('-----STATE------')
            # print('-----STOP------')           

        
        elif completion[0] == 1: #start/restart the motion
            self.dx = completion[1]
            # print('-----STATE------')
            # print('-----START------')               
        
        elif completion[0]==2:
            self.break_flag=self.previous_value
            self.previous_value=self.break_flag
        self.previous_value=self.break_flag
                   
        # np.savetxt('yogesh/new.txt',keyboard_input)
        
        # 1. stop and start again based on human language
        # 2. 
        
        
        # print(msg.data)
        if (keyboard_input=='w'):
            print("hahahah")
            x=-0.01
            y=0
        elif (keyboard_input=='s'):
            x=0.01
            y=0
        elif (keyboard_input=='d'):
            x=0
            y=0.01
        elif (keyboard_input=='a'):
            x=0
            y=-0.01    
        elif(keyboard_input=='e'):
            x=0
            y=0
        # print(x,y)
        if (keyboard_input=='h'):
            self.alpha=1
        elif (keyboard_input=='r'):
            self.alpha=0
        
        self.human_policy = np.array([x,y])
         
        # *****************************************
    
        
    def shared_control_callback(self, dummy_msg):
        if np.sqrt((self.ee_curr_pos[0]-self.ee_goal_pos[0])**2+(self.ee_curr_pos[1]-self.ee_goal_pos[1])**2)<0.05:
            rospy.loginfo("Task completed. iter: {}, path length: {}".format(self.iter, self.path_len))
            return
        # *****************************************
        # ************ To be done here ************
        # * policy is a weighted sum of robot policy and human policy
        # policy = np.zeros(2)
        # policy=np.array([0.005,0.005])#just for test
        
        alpha=self.alpha
        # alpha=  #for setting up manually
        robot_policy=self.get_robot_policy()
        self.dx = 0
        human_policy=self.human_policy
        
        policy= alpha*robot_policy+ (1-alpha)*human_policy 
        # policy=robot_policy
        # *****************************************
        # print(np.linalg.norm(policy))
        if np.linalg.norm(policy) > 0.09:
            rospy.loginfo("Policy is too fast. Robot is paused.")
            # policy = np.zeros(2)
        # print(self.ee_next_pos)    
        self.iter += 1  
        self.ee_next_pos[0] = self.ee_next_pos[0] + policy[0]
        self.ee_next_pos[1] = self.ee_next_pos[1] + policy[1]
        self.ee_next_pos[2] = self.ee_z
        self.path.append(self.ee_next_pos)
        self.path_len += np.linalg.norm(policy)
        self.move_to_position()
    
    

    def get_robot_policy(self):
        # *****************************************
        # ************ To be done here ************
        robot_policy = np.zeros(2)
        if self.dx == 0:
            dx_dy=self.ee_goal_pos-self.ee_curr_pos
            dx_dy=dx_dy[:2]
            # print(dx_dy)
            distance=np.linalg.norm(dx_dy)# actual distance to be moved
            small_division=distance/0.01
            robot_policy=dx_dy/small_division
        else: 
            self.ee_goal_pos = np.array([self.ee_goal_pos[0] - self.dx, self.ee_goal_pos[1], self.ee_goal_pos[2]])
            dy = self.ee_goal_pos[1]-self.ee_curr_pos[1]
            small_division=dy/0.01
            robot_policy=np.array([-self.dx,dy/small_division])
            # print(robot_policy)


        
        # *****************************************
        return robot_policy
    
    def go_to_start_pos(self):
        self.ee_next_pos = self.ee_start_pos
        self.move_to_position()

    def position_callback(self, msg):
        self.thetas[0] = msg.position[0] 
        self.thetas[1] = msg.position[1]
        self.thetas[2] = msg.position[2]
        self.thetas[3] = msg.position[3] 
        self.thetas[4] = msg.position[4]
        self.thetas[5] = msg.position[5]
        curr = forward_k(self.thetas[0]- PI, self.thetas[1], self.thetas[2], self.thetas[3]+ (0.5*PI), self.thetas[4], self.thetas[5])
        self.ee_curr_pos = curr[0:3,3]*0.001 # get end-effector current position in meters

    
    

    def move_to_position(self):
        spin_count = 0
        at_goal = 0
        joint_state = ur_invk(self.ee_next_pos[0], self.ee_next_pos[1], self.ee_next_pos[2], 0)
        driver_msg = command()
        driver_msg.destination = joint_state
        driver_msg.v = 0.5
        driver_msg.a = 0.5
        self.pub_cmd.publish(driver_msg)
        self.loop_rate.sleep()
        
        while at_goal == 0:
            # print(self.break_flag)
            if self.break_flag == 1:
                continue
            
            if (
                abs(self.thetas[0] - driver_msg.destination[0]) < 0.005 and
                abs(self.thetas[1] - driver_msg.destination[1]) < 0.005 and
                abs(self.thetas[2] - driver_msg.destination[2]) < 0.005 and
                abs(self.thetas[3] - driver_msg.destination[3]) < 0.005 and
                abs(self.thetas[4] - driver_msg.destination[4]) < 0.005 and
                abs(self.thetas[5] - driver_msg.destination[5]) < 0.005
            ):
                at_goal = 1
                self.pub_control_clock.publish("dummy message")
                return
            
            self.loop_rate.sleep()
            
            if spin_count > self.SPIN_RATE * 5:
                self.pub_cmd.publish(driver_msg)
                rospy.loginfo("Just published again driver_msg")
                spin_count = 0
            
            spin_count += 1
    


def main():
    
    sc = SharedControl()
   
    rospy.spin()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
