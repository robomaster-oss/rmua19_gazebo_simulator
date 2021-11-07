#!/usr/bin/python3
import sys
import threading 
import numpy as np
import cv2
import rclpy
import copy
import time

from sensor_msgs.msg import Image
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd

from flask import Flask, render_template, request,Response
from flask_cors import CORS
from flask_socketio import SocketIO, emit

from engineio.payload import Payload

Payload.max_decode_packets = 100
################################
# ROS2 
################################
def publishChassisCmdMsg(pub,x,y,w):
    msg = ChassisCmd()
    msg.twist.linear.x = x
    msg.twist.linear.y = y
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = w
    pub.publish(msg)

def publishGimbalCmdMsg(pub,pitch,yaw):
    msg = GimbalCmd()
    msg.position.yaw=yaw
    msg.position.pitch=pitch
    pub.publish(msg)

def publishShootCmdMsg(pub,num,vel):
    msg = ShootCmd()
    msg.projectile_num=num
    msg.projectile_velocity=vel
    pub.publish(msg)

event = threading.Event()
def img_callback(msg : Image):
    global frame
    np_img = np.reshape(msg.data, (msg.height, msg.width, 3)).astype(np.uint8)
    img=cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
    #encoded_img = cv2.imencode(".jpg", img)
    frame = cv2.imencode(".jpg",img)[1].tobytes()
    event.set()

## Thread for spin (image subscription)
def ros_img_thread(node,robot_name):
    img_sub = node.create_subscription(Image, '/%s/front_camera/image'%(robot_name),img_callback,rclpy.qos.qos_profile_sensor_data)
    rclpy.spin(node)
    node.destroy_node()

################################
# Flask
################################
app = Flask(__name__)
app.config['SECRET_KEY'] = 'randonlygeneratedkeys'
CORS(app)
socketio = SocketIO(app,async_mode='threading')

def gen():
    while True:
        event.wait()
        event.clear()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')       
        
@app.route("/",  methods=['GET'])
def login():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    #pass
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR")
    print(e)
    print(request.event["message"])
    print(request.event["args"])
    print("======================= ")

##global variables
chassis_x=0.0
chassis_y=0.0
xy_coeff=1.0
gimbal_pitch=0.0
gimbal_yaw=0.0
shoot_flag=False
def getChassisXY(gp_data):
    x=0.0
    y=0.0
    if gp_data['UP']:
        x=x+1
    if gp_data['DOWN']:
        x=x-1
    if gp_data['LEFT']:
        y=y+1
    if gp_data['RIGHT']:
        y=y-1 
    return x,y

@socketio.on('control', namespace='/control')
def control_message(message):
    global chassis_x,chassis_y,gimbal_pitch,gimbal_yaw,shoot_flag,xy_coeff
    gp_data=message['gp_data']
    #chassis
    x,y=getChassisXY(gp_data)
    if gp_data['LT'] and (xy_coeff < 2.0): 
        xy_coeff = xy_coeff + 0.1
    if gp_data['RT'] and (coeff > 0.1): 
        xy_coeff = xy_coeff - 0.1
    chassis_x,chassis_y=xy_coeff*x,xy_coeff*y
    #gimbal
    gimbal_pitch=float(gp_data['RS_ax1'])
    if (abs(float(gp_data['RS_ax0'])) > 0.001):
        gimbal_yaw = gimbal_yaw+float(gp_data['RS_ax0'])
    #shoot
    shoot_flag = gp_data['RB']
    #print("msg:",chassis_x,chassis_y,gimbal_pitch,gimbal_yaw,shoot_flag)



def ros_process_thread(node,robot_name):
    global chassis_x,chassis_y,gimbal_pitch,gimbal_yaw,shoot_flag
    chassis_cmd_pub = node.create_publisher(ChassisCmd,'/%s/robot_base/chassis_cmd'%(robot_name) , 10)
    gimbal_cmd_pub = node.create_publisher(GimbalCmd, '/%s/robot_base/gimbal_cmd'%(robot_name) , 10)
    shoot_cmd_pub = node.create_publisher(ShootCmd, '/%s/robot_base/shoot_cmd'%(robot_name), 10)
    while rclpy.ok():
        #publish
        publishChassisCmdMsg(chassis_cmd_pub,chassis_x,chassis_y,0.0)
        publishGimbalCmdMsg(gimbal_cmd_pub,gimbal_pitch,gimbal_yaw)
        #shoot
        if shoot_flag: 
            publishShootCmdMsg(shoot_cmd_pub,1,20)
        #print("msg2:",chassis_x,chassis_y,gimbal_pitch,gimbal_yaw,shoot_flag)
        time.sleep(0.1)


if __name__ == "__main__":
    #ROS2 Task
    rclpy.init()
    node = rclpy.create_node('player_web')
    #standard_robot_red1,standard_robot_blue1
    robot_name="standard_robot_red1"
    port=5000
    if len(sys.argv) == 3:
        robot_name = str(sys.argv[1])
        port = int(sys.argv[2])
    print("Robot name:",robot_name, "Port:",port)
    img_thread = threading.Thread(target=ros_img_thread, args=(node,robot_name))
    img_thread.start()
    process_thread = threading.Thread(target=ros_process_thread, args=(node,robot_name))
    process_thread.start()
    #Flask Task
    socketio.run(app,debug = True, host="0.0.0.0", port = port)
   