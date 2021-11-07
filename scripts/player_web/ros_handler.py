import sys
import threading
import numpy as np
import cv2
import rclpy
import copy
import time
import base64

from sensor_msgs.msg import Image
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd




def publish_chassis_cmd_msg(pub, x, y, w):
    # print('pub chassis')
    msg = ChassisCmd()
    msg.twist.linear.x = x
    msg.twist.linear.y = y
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = w
    # print(msg)
    pub.publish(msg)


def publish_gimbal_cmd_msg(pub, pitch, yaw):
    # print('pub gimbal')
    msg = GimbalCmd()
    msg.position.yaw = yaw
    msg.position.pitch = pitch
    # print(msg)
    pub.publish(msg)


def publish_shoot_cmd_msg(pub, num, vel):
    # print('pub shoot')
    msg = ShootCmd()
    msg.projectile_num = num
    msg.projectile_velocity = vel
    # print(msg)
    pub.publish(msg)


# def img_callback(send_img_function):
#     def func(msg: Image):
#         np_img = np.reshape(msg.data, (msg.height, msg.width, 3)).astype(np.uint8)
#         img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
#         image = cv2.imencode('.jpg', img)[1]
#         image_code = str(base64.b64encode(image))[2:-1]
#         send_img_function(image_code)
#
#     return func
#
#
# def ros_img_thread(node, robot_name, send_img_function):
#     img_sub = node.create_subscription(Image, '/%s/front_camera/image' % (robot_name), img_callback(send_img_function),
#                                        rclpy.qos.qos_profile_sensor_data)
#     rclpy.spin(node)
#     node.destroy_node()



# def ros_process_thread(node,robot_name):
#     global chassis_x,chassis_y,gimbal_pitch,gimbal_yaw,shoot_flag
#     chassis_cmd_pub = node.create_publisher(ChassisCmd,'/%s/robot_base/chassis_cmd'%(robot_name) , 10)
#     gimbal_cmd_pub = node.create_publisher(GimbalCmd, '/%s/robot_base/gimbal_cmd'%(robot_name) , 10)
#     shoot_cmd_pub = node.create_publisher(ShootCmd, '/%s/robot_base/shoot_cmd'%(robot_name), 10)
#     while rclpy.ok():
#         #publish
#         publishChassisCmdMsg(chassis_cmd_pub,chassis_x,chassis_y,0.0)
#         publishGimbalCmdMsg(gimbal_cmd_pub,gimbal_pitch,gimbal_yaw)
#         #shoot
#         if shoot_flag:
#             publishShootCmdMsg(shoot_cmd_pub,1,20)
#         #print("msg2:",chassis_x,chassis_y,gimbal_pitch,gimbal_yaw,shoot_flag)
#         time.sleep(0.1)

# def send_instruction(chassis_x, chassis_y, gimbal_pitch, gimbal_yaw, shoot):
#     if shoot:
#         publish_shoot_cmd_msg(shoot_cmd_pub, 1, 20)
#     publish_chassis_cmd_msg(chassis_cmd_pub, chassis_x, chassis_y, 0.0)
#     publish_gimbal_cmd_msg(gimbal_cmd_pub, gimbal_pitch, gimbal_yaw)