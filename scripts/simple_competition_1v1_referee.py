#!/usr/bin/python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32, String

def parse_attack_info(attack_str):
    # parse msg
    info = attack_str.split(',')
    if len(info) < 2:
        return None
    # shooter info
    shooter = info[0].split('/')
    if len(shooter) != 2:
        return None
    shooter_model_name = shooter[0]
    shooter_name = shooter[1]
    # target info
    target = info[1].split('/')
    if len(target) < 6:
        return None
    target_model_name = target[-5]
    target_link_name = target[-3]
    target_collision_name = target[-1]
    # print("target info:",target)
    if target_collision_name != 'target_collision':
        return None
    return shooter_model_name, shooter_name, target_model_name, target_link_name

class SimpleRefereeSystem(Node):
    def __init__(self):
        super().__init__('referee_system')
        self.attack_info_sub = self.create_subscription(
            String,
            '/referee_system/attack_info',
            self.attack_info_callback,
            50)
        self.reset_sub = self.create_subscription(
            Int32,
            '/referee_system/reset',
            self.reset_callback,
            1)
        self.red1_hp_pub = self.create_publisher(Int32, '/referee_system/standard_robot_red1/hp', 10)
        self.blue1_hp_pub = self.create_publisher(Int32, '/referee_system/standard_robot_blue1/hp', 10)
        self.hp_timer = self.create_timer(0.5, self.hp_timer_callback)
        self.attack_info_sub  # prevent unused variable warning
        self.hp_timer  # prevent unused variable warning
        self.red1_hp = 500
        self.blue1_hp = 500
        self.game_over = False

    def attack_info_callback(self, msg: String):
        # parse msg
        data = parse_attack_info(msg.data)
        if data is None:
            return
        target_model_name = data[2]
        target_link_name = data[3]
        # process attack info
        if self.game_over:
            return
        if 'armor' in target_link_name:
            if 'red1' in target_model_name and self.red1_hp > 0:
                self.red1_hp = self.red1_hp - 10
            if 'blue1' in target_model_name and self.blue1_hp > 0:
                self.blue1_hp = self.blue1_hp - 10
        if self.red1_hp == 0 or self.blue1_hp == 0:
            self.game_over = True

    def hp_timer_callback(self):
        msg = Int32()
        msg.data = self.red1_hp
        self.red1_hp_pub.publish(msg)
        msg.data = self.blue1_hp
        self.blue1_hp_pub.publish(msg)
        # print("HP>>> red1:%d, blue1:%d"%(self.red1_hp,self.blue1_hp))

    def reset_callback(self, msg: Int32):
        self.red1_hp = msg.data
        self.blue1_hp = msg.data
        self.game_over = False
        

def main(args=None):
    rclpy.init(args=args)
    referee_system = SimpleRefereeSystem()
    rclpy.spin(referee_system)
    referee_system.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()