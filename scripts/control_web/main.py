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
from rmoss_interfaces.msg import RefereeCmd
from rmoss_interfaces.msg import RobotStatus
import time

# ==========================================
#    robot list
# ==========================================
robot_names = ['red_standard_robot1', 'blue_standard_robot1']
node = None

# ==========================================
#    cmd constant
# ==========================================
control_map = {
    'PREPARATION' : 0,    # system cmd: 3 min preparation (free control for robot, disable referee system)
    'SELF_CHECKING' : 1,  # system cmd: referee system self-checking (unable to control robot, and 
                            # init all referee system data).
    'START_GAME' : 2,     # system cmd: start game (free control for robot, enable referee system)
    'STOP_GAME' : 3,      # system cmd: stop game  (unable to control robot, disable referee system)
    'KILL_ROBOT' : 4,     # control robot: kill robot with robot_name
    'REVIVE_ROBOT' : 5   # control robot: revive robot with robot_name
}

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
#    namespace class
# ==========================================
class RefereeSocketHandler(Namespace):
    def __init__(self, namespace):
        super().__init__(namespace=namespace)
        self.robot_name = namespace[1:]
        self.info_thread = None

        # ==========================================
        #   pubs
        # ==========================================
        self.referee_pub = node.create_publisher(RefereeCmd, '/referee_system/referee_cmd', 10)


    def on_connect(self):
        global info_thread, node, robot_names, chosen_robot_dict
        emit('robot_names', {'list': robot_names})
        with thread_lock:
            if info_thread is None:
                info_thread = socketio.start_background_task(ros_info_thread, node)


    def on_control(self, message):
        
        robot_name = message['robot_name']
        instruction = message['instruction']
        cmd = control_map[instruction]
        msg = RefereeCmd()
        msg.cmd = cmd
        msg.robot_name = robot_name
        self.referee_pub.pub(msg)
        
        

    def on_default_error_handler(self, e):
        print("======================= ERROR =======================")
        print(e)
        print(request.event["message"])
        print(request.event["args"])
        print("=====================================================")
    

# =============================================================================================================================

# =====================================================================  funcitons   ==========================================

# ==========================================
#    发送血量、弹药量、图像数据给前端
# ==========================================

        # 'name': 'string',
        # 'remain_hp': 'int32',
        # 'max_hp': 'int32',
        # 'total_projectiles': 'int32',
        # 'used_projectiles': 'int32',
        # 'hit_projectiles': 'int32',
def send_refere_info_callback( robot_name):
    def func(message):
        # print(message)
        socketio.emit('robot_status', {'robot_name':robot_name, 
        'remain_hp': message.remain_hp,
        'max_hp':message.max_hp,
        'total_projectiles':message.total_projectiles,
        'used_projectiles':message.used_projectiles,
        'hit_projectiles':message.hit_projectiles
        })
        # if kind == BLUE_HP:
        #     socketio.emit('blue_hp', {'value': message.remain_hp}, namespace='/'+robot_name)
        # elif kind == RED_HP:
        #     socketio.emit('red_hp', {'value': message.remain_hp}, namespace='/'+robot_name)
        # else:    
        #     socketio.emit('attack', {'value': message.remain_hp}, namespace='/'+robot_name)

    return func

# ==========================================
#    信息发送线程
# ==========================================
def ros_info_thread(node): 
    global BLUE_HP, RED_HP, ATTACK_INFO
    for robot_name in robot_names:
        hp_sub = node.create_subscription(RobotStatus, '/referee_system/%s/robot_status' % (robot_name), send_refere_info_callback(robot_name),
                                        10)
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
node=rclpy.create_node('referee_web')

# ==========================================
#    init namespace class
# ==========================================
socketio.on_namespace(RefereeSocketHandler('/'))

if __name__ == '__main__':
    port=5000
    if len(sys.argv) == 3:
        robot_name = str(sys.argv[1])
        port = int(sys.argv[2])
    socketio.run(app, host='0.0.0.0', port=2350)
