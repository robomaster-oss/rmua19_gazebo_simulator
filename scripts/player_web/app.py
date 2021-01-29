#!/usr/bin/python3
import sys
import threading 
import numpy as np
import cv2
import rclpy

from sensor_msgs.msg import Image
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd

from flask import Flask, render_template, request,Response
from flask_cors import CORS
from flask_socketio import SocketIO, emit

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
def spin_thread(node):
    rclpy.spin(node)
    node.destroy_node()

################################
# Flask
################################
app = Flask(__name__)
app.config['SECRET_KEY'] = 'randonlygeneratedkeys'
CORS(app)
socketio = SocketIO(app)

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
    return Response(gen(), mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on_error_default
def default_error_handler(e):
    print("======================= ERROR")
    print(e)
    print(request.event["message"])
    print(request.event["args"])
    print("======================= ")
    

@socketio.on('control', namespace='/control')
def control_message(message):
    gp_data=message['gp_data']
    #chassis
    y=-gp_data['LS_ax0']
    x=-gp_data['LS_ax1']
    w=0
    if gp_data['LT']: 
        w=1
    if gp_data['RT']: 
        w=-1
    x=1.0*x
    y=1.0*y
    w=1.0*w
    publishChassisCmdMsg(chassis_cmd_pub,x,y,w)
    #gimbal
    yaw=-float(gp_data['RS_ax0'])
    pitch=float(gp_data['RS_ax1'])
    publishGimbalCmdMsg(gimbal_cmd_pub,pitch,yaw)
    #shoot
    if  gp_data['RB']: 
        publishShootCmdMsg(shoot_cmd_pub,1,20)
    #print("msg",x,y,w,pitch,yaw,gp_data['RB'])

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
    chassis_cmd_pub = node.create_publisher(ChassisCmd,'/%s/robot_base/chassis_cmd'%(robot_name) , 10)
    gimbal_cmd_pub = node.create_publisher(GimbalCmd, '/%s/robot_base/gimbal_cmd'%(robot_name) , 10)
    shoot_cmd_pub = node.create_publisher(ShootCmd, '/%s/robot_base/shoot_cmd'%(robot_name), 10)
    img_sub = node.create_subscription(Image, '/%s/front_camera/image'%(robot_name),img_callback,rclpy.qos.qos_profile_sensor_data)
    img_thread = threading.Thread(target=spin_thread, args=(node,))
    img_thread.start()
    #Flask Task
    socketio.run(app,debug = True, host="0.0.0.0", port = port)
   