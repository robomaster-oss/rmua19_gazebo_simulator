# =====================================================================  libs   ==========================================

# ==========================================
#    flask
# ==========================================
from threading import Lock
from flask import Flask, render_template, session, request, \
    copy_current_request_context
from flask.json import tojson_filter
from flask_socketio import SocketIO, emit, join_room, leave_room, \
    close_room, rooms, disconnect, Namespace
from engineio.payload import Payload
from flask_cors import CORS
import logging

# ==========================================
#   packages for ros
# ==========================================
import rclpy
import sys
from sensor_msgs.msg import Image
import cv2
import base64
import numpy as np
from std_msgs.msg import Int32
from rmoss_interfaces.msg import ChassisCmd
from rmoss_interfaces.msg import GimbalCmd
from rmoss_interfaces.msg import ShootCmd
import time

# ==========================================
#    ros functions
# ==========================================
from ros_handler import *

# ==========================================
#    robot list
# ==========================================
robot_names = ['standard_robot_red1', 'standard_robot_blue1']
chosen_robot_dict = {}
node = None

# ==========================================
#    constant
# ==========================================
BLUE_HP=0
RED_HP=1
ATTACK_INFO=2

# ==========================================
#    用于调试
# ==========================================
counter = 0
start_time = 0

# ==========================================
#    config
# ==========================================
async_mode = "threading"
Payload.max_decode_packets = 10000
app = Flask(__name__)
app.config['SECRET_KEY'] = 'secret!'
CORS(app)
socketio = SocketIO(app, async_mode=async_mode)
info_thread = None
thread_lock = Lock()
log = logging.getLogger('werkzeug')
log.setLevel(logging.ERROR)

# =====================================================================  Classes   ==========================================
# ==========================================
#    handle each robot player
# ==========================================
class RobotSocketHandler(Namespace):
    def __init__(self, namespace):
        super().__init__(namespace=namespace)
        self.robot_name = namespace[1:]
        
        print("self.robot_name:"+self.robot_name)
        # self.node = None
        self.info_thread = None

        # ==========================================
        #   some states
        # ==========================================
        self.counter = 0
        self.start_time = 0
        self.gimbal_pitch = 0.0
        self.gimbal_yaw = 0.0
        self.speed = 1.0
        self.yaw_upper_bound = 1.37
        self.yaw_lower_bound = -1.37
        self.pitch_upper_bound = 1.1
        self.pitch_lower_bound = -0.9

        # ==========================================
        #   pubs
        # ==========================================
        self.chassis_cmd_pub = node.create_publisher(ChassisCmd, '/%s/robot_base/chassis_cmd' % (robot_name), 10)
        self.gimbal_cmd_pub = node.create_publisher(GimbalCmd, '/%s/robot_base/gimbal_cmd' % (robot_name), 10)
        self.shoot_cmd_pub = node.create_publisher(ShootCmd, '/%s/robot_base/shoot_cmd' % (robot_name), 10)


    def on_connect(self):
        global info_thread, node, robot_names, chosen_robot_dict
        chosen_robot_dict[self.robot_name] = True
        with thread_lock:
            if info_thread is None:
                info_thread = socketio.start_background_task(ros_info_thread, node, robot_name)

    def on_control(self, message):
        chassis_x = 0.0
        chassis_y = 0.0
        shoot = False 
        if message['w']:
            chassis_x = chassis_x + 1*self.speed
        if message['s']:
            chassis_x = chassis_x - 1*self.speed
        if message['a']:
            chassis_y = chassis_y + 1*self.speed
        if message['d']:
            chassis_y = chassis_y - 1*self.speed
        if message['q']:
            self.speed = min((self.speed + 1), 5.0)
        if message['e']:
            self.speed = max((self.speed - 1), 1.0)
        if message['shoot']:
            shoot = True
        if message['reset']:
            reset_hp()
        movement_yaw = message['movementX']
        movement_pitch = message['movementY']
        if movement_pitch<=0:
            self.gimbal_pitch = max(self.gimbal_pitch + movement_pitch, self.pitch_lower_bound)
        else:
            self.gimbal_pitch = min(self.gimbal_pitch + movement_pitch, self.pitch_upper_bound)
        self.gimbal_yaw = self.gimbal_yaw + movement_yaw
        send_instruction(chassis_x, chassis_y, self.gimbal_pitch, self.gimbal_yaw, shoot,self.shoot_cmd_pub, self.chassis_cmd_pub, self.gimbal_cmd_pub)

    def on_default_error_handler(self, e):
        print("======================= ERROR =======================")
        print(e)
        print(request.event["message"])
        print(request.event["args"])
        print("=====================================================")
    
    def on_disconnect(self):
        # print['1']
        chosen_robot_dict[self.robot_name] = False
        print(chosen_robot_dict)
class BaseSocketHandler(Namespace):
    # ==========================================
    #    连接事件
    # ==========================================
    def on_connect(self):
    #     global info_thread, node, robot_names, chosen_robot_dict
    #     with thread_lock:
    #         if info_thread is None:
    #             info_thread = socketio.start_background_task(ros_info_thread, node, robot_name)
        emit('robot_names', {'list': robot_names, 'chosen': chosen_robot_dict})


    # ==========================================
    #    延迟
    # ==========================================
    def on_ping(self):
        emit('pong')


    # ==========================================
    #    错误处理
    # ==========================================
    def on_default_error_handler(self, e):
        print("======================= ERROR =======================")
        print(e)
        print(request.event["message"])
        print(request.event["args"])
        print("=====================================================")


# =============================================================================================================================

# =====================================================================  funcitons   ==========================================

# ==========================================
#    发送控制指令
# ==========================================
def send_instruction(chassis_x, chassis_y, gimbal_pitch, gimbal_yaw, shoot, shoot_cmd_pub, chassis_cmd_pub, gimbal_cmd_pub):
    if shoot:
        publish_shoot_cmd_msg(shoot_cmd_pub, 1, 20)
    publish_chassis_cmd_msg(chassis_cmd_pub, chassis_x, chassis_y, 0.0)
    publish_gimbal_cmd_msg(gimbal_cmd_pub, gimbal_pitch, gimbal_yaw)

# ==========================================
#       重置血量
# ==========================================
def reset_hp():
    publish_reset_cmd_msg(reset_cmd_pub)

# ==========================================
#    发送血量、弹药量、图像数据给前端的线程
# ==========================================
def send_img_callback(robot_name):
    def func(msg: Image):
        # ==========================================
        #   发送帧率代码计算
        # 
        # global counter, start_time
        # if start_time == 0 or time.time() - start_time >1:
        #     print(counter)
        #     counter = 0
        #     start_time = time.time()
        # counter = counter + 1
        # ==========================================
        np_img = np.reshape(msg.data, (msg.height, msg.width, 3)).astype(np.uint8)
        img = cv2.cvtColor(np_img, cv2.COLOR_RGB2BGR)
        image = cv2.imencode('.jpg', img)[1]
        image_code = str(base64.b64encode(image))[2:-1]
        socketio.emit('image', {'img': image_code}, namespace='/'+robot_name)

    return func

# ==========================================
#    发送血量、弹药量、图像数据给前端
# ==========================================
def send_refere_info_callback(kind, robot_name):
    global BLUE_HP, RED_HP, ATTACK_INFO
    def func(message):
        if kind == BLUE_HP:
            socketio.emit('blue_hp', {'value': message.data}, namespace='/'+robot_name)
        elif kind == RED_HP:
            socketio.emit('red_hp', {'value': message.data}, namespace='/'+robot_name)
        else:    
            socketio.emit('attack', {'value': message.data}, namespace='/'+robot_name)

    return func

# ==========================================
#    信息发送线程
# ==========================================
def ros_info_thread(node, robot_name): 
    global BLUE_HP, RED_HP, ATTACK_INFO
    for robot_name in robot_names:
        img_sub = node.create_subscription(Image, '/%s/front_camera/image' % (robot_name), send_img_callback(robot_name),
                                        45)
        blue_hp_sub = node.create_subscription(Int32, '/referee_system/standard_robot_blue1/hp', send_refere_info_callback(BLUE_HP, robot_name),
                                        10)
        red_hp_sub = node.create_subscription(Int32, '/referee_system/standard_robot_red1/hp', send_refere_info_callback(RED_HP, robot_name),
                                        10)
    # attack_info_sub = node.create_subscription(String, '/referee_system/attack_info', send_refere_info_callback(ATTACK_INFO),
    # .                                    10)
    rclpy.spin(node)
    node.destroy_node()


# ==========================================
#    返回页面
# ==========================================
@app.route('/')
def index():
    return render_template('index.html', async_mode=socketio.async_mode)

# ============================================================================================================================
# =====================================================================  others   ============================================


# ==========================================
#    init node
# ==========================================
rclpy.init()
node=rclpy.create_node('player_web')

# ==========================================
#    init namespace class
# ==========================================
socketio.on_namespace(BaseSocketHandler('/'))
for robot_name in robot_names:
    print('robot_name:'+robot_name)
    socketio.on_namespace(RobotSocketHandler('/'+robot_name))
reset_cmd_pub = node.create_publisher(Int32, '/referee_system/reset', 10)

if __name__ == '__main__':
    port=5000
    if len(sys.argv) == 3:
        robot_name = str(sys.argv[1])
        port = int(sys.argv[2])
    socketio.run(app, host='0.0.0.0')
