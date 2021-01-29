#!/usr/bin/python3
import threading 
import numpy as np
import rclpy
from std_msgs.msg import String

red1_hp=500
blue1_hp=500
game_over = False
is_debug = False

def attack_info_callback(msg : String):
    #parse msg
    info=msg.data.split(">")
    if len(info)<2:
        return
    #shooter info
    shooter = info[0].split("/")
    if len(shooter)!=2:
        return
    shooter_model_name=shooter[0]
    shooter_name=shooter[1]
    #target info
    target=info[1].split("/")
    if len(target)<6:
        return
    target_model_name=target[-5]
    target_link_name=target[-3]
    target_collision_name=target[-1]
    #process attack info
    global game_over,red1_hp,blue1_hp
    if game_over:
        return
    #print("target info:"+target_model_name+"/"+target_link_name+"/"+target_collision_name)
    if target_collision_name=="target_collision" and "armor" in target_link_name:
        if "red1" in target_model_name  and red1_hp>0:
            red1_hp=red1_hp-10
        if "blue1" in target_model_name and blue1_hp>0:
            blue1_hp=blue1_hp-10  
    if red1_hp==0 or blue1_hp==0:
        game_over = True

def timer_callback():
    global red1_hp,blue1_hp,game_over
    if not game_over:
        print("HP>>> red1:%d, blue1:%d"%(red1_hp,blue1_hp))
    else:
        print("HP>>> red1:%d, blue1:%d"%(red1_hp,blue1_hp))
        if blue1_hp == 0:
            print("red win!")
        else:
            print("blue win!")
        exit(0)
    

def main():
    rclpy.init()
    node = rclpy.create_node('referee_system')
    attack_info_sub = node.create_subscription(String, '/referee_system/attack_info',attack_info_callback,rclpy.qos.qos_profile_system_default)
    display_timer = node.create_timer(2.0, timer_callback)
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()


   